#include "platform.h"
#include "applications.h"
#include "oled.h"
#include "imu.h"
#include "mavlink.h"
#include "datatypes.h"
#include "LowPassFilter.h"
#include "LowPassFilter2p.h"
#include "FreeRTOS.h"
#include "task.h"
#include "pid.h"

static TaskHandle_t handle_startup;
mavlink_system_t mavlink_system;
static uint8_t mavlink_send_buffer[256];
static uint32_t mavlink_send_pos = 0;
static DEV_uart *mavlink = &serial_1;
//static DEV_usb *shell = &usb;
IMU imu;
SemaphoreHandle_t sample_sync = NULL;
static float gyro[3];
static float attitude[3];
static float target_attitude[3];
static float target_speed;
static float target_speed_angle;
static float attitude_ctrl_output[3];
static float enc_ctrl_output[3];
PID pid_gyro[3];
PID pid_attitude[3];
PID pid_enc[2];
static bool is_stable;

volatile int DebugCnt;

extern "C"
{
	extern void user_loop(void);
}

void startup_task(void *argument);
void test_task(void *argument);
void imu_task(void *argument);
void control_task(void *argument);
void oled_task(void *argument);
void periperal_task(void *argument);
void battery_task(void *argument);
void sonic_task(void *argument);
void serial_task(void *argument);
void monitor_task(void *argument);
static void mavlink_proc(mavlink_message_t &msg);
void attitude_control_task(void *argument);
void gyro_control_task(void *argument);
void imu_sample_task(void *argument);
void enc_control_task(void *argument);

void user_loop(void)
{
	xTaskCreate(startup_task, "Startup", 1024, NULL, RTOS_PRIORITY_HIGH, &handle_startup);
	vTaskStartScheduler();
	while (1)
		;
}

void startup_task(void *argument)
{
	//	sgw_param_load_default();

	platform_init();
	
	imu.init();	

	mavlink_system.sysid = 0x21;
	mavlink_system.compid = 0;

	//	sgw_init();

	// task
	xTaskCreate(imu_sample_task, "IMU sample", 256, NULL, RTOS_PRIORITY_HIGH, NULL);	
	xTaskCreate(imu_task, "IMU", 256, NULL, RTOS_PRIORITY_HIGH, NULL);
	xTaskCreate(gyro_control_task, "gyro ctrl", 256, NULL, RTOS_PRIORITY_NORMAL, NULL);
	xTaskCreate(attitude_control_task, "attitude ctrl", 256, NULL, RTOS_PRIORITY_NORMAL, NULL);
	xTaskCreate(enc_control_task, "enc ctrl", 256, NULL, RTOS_PRIORITY_NORMAL, NULL);

	xTaskCreate(serial_task, "Serial", 512, NULL, RTOS_PRIORITY_NORMAL, NULL);
	//	xTaskCreate(sonic_task, "Sonic", 256, NULL, RTOS_PRIORITY_NORMAL, NULL);
	//	xTaskCreate(oled_task, "Oled", 256, NULL, RTOS_PRIORITY_LOWEST, NULL);
	//	xTaskCreate(battery_task, "Battery", 256, NULL, RTOS_PRIORITY_NORMAL, NULL);
	//xTaskCreate(periperal_task, "Periperal", 512, NULL, RTOS_PRIORITY_NORMAL, NULL);

	//#ifdef DEBUG
	xTaskCreate(test_task, "Test", 1024, NULL, RTOS_PRIORITY_HIGH, NULL);
//	xTaskCreate(monitor_task, "Monitor", 256, NULL, RTOS_PRIORITY_NORMAL, NULL);
//#endif

	vTaskDelete(handle_startup);
}

volatile float s, s_lp, s_lp_2;
int32_t speed[2];

void test_task(void *argument)
{
	float i = 0;
	TickType_t tick_abs;

	tick_abs = xTaskGetTickCount();
	
	for (;;)
	{

		vTaskDelay(2);
	}
}

void imu_sample_task(void *argument)
{
	TickType_t tick_abs;
	tick_abs = xTaskGetTickCount();

	sample_sync = xSemaphoreCreateBinary();

	for (;;)
	{
		if (xSemaphoreTake(sample_sync, portMAX_DELAY) == pdTRUE)
		{
			// push sensor raw data to queue
			imu.sample();
		}
	}
}

void imu_task(void *argument)
{
	TickType_t tick_abs;
	tick_abs = xTaskGetTickCount();

	for (;;)
	{
		// will send a message to gyro ctrl thread
		imu.handle();

		// vTaskDelay(1);
	}
}

typedef struct
{
	float pwm_out;
	float last_pwm_out;

	float gyro_ctrl_out;
	float speed_ctrl_out;

} pwm_t;

pwm_t pwm_1, pwm_2;

bool pwm_calculate(pwm_t *pwm)
{
	pwm->pwm_out = constrain_float(pwm->pwm_out,
								   pwm->last_pwm_out - 0.01, pwm->last_pwm_out + 0.01);
	pwm->last_pwm_out = pwm->pwm_out;

	return true;
}

void gyro_control_task(void *argument)
{
	TickType_t tick_abs;

	tick_abs = xTaskGetTickCount();

	pid_gyro[PITCH].init(3.5, 0, 0,
						 0.001,
						 0,
						 0, 1,
						 1.0f / 1000);

	pid_gyro[YAW].init(3, 0, 0,
					   0.001,
					   0,
					   0, 1,
					   1.0f / 1000);

	for (;;)
	{
		volatile static float error, output_pitch, output_yaw;
		float out_1, out_2;

		// "loose coupling" here, enable to make it "tight coupling"
		if (imu.gyro_even())
		{
			imu.gyroGet(gyro);

			error = attitude_ctrl_output[PITCH] - gyro[PITCH];
			output_pitch = -pid_gyro[PITCH].apply(error);

			error = 0 - gyro[YAW];
			output_yaw = -pid_gyro[YAW].apply(error);

//			pwm_output[0] = output_pitch + output_yaw;
//			pwm_output[1] = output_pitch - output_yaw;

//			// TODO: saturate?
//			if (pwm_output[0] > 1.0 || pwm_output[1] > 1.0)
//			{
//				
//			}
			
//			motor[0].write(pwm_1.pwm_out);
//			motor[1].write(pwm_2.pwm_out);

			//DebugCnt++;
		}
	}
}

void attitude_control_task(void *argument)
{
	TickType_t tick_abs;

	tick_abs = xTaskGetTickCount();

	pid_attitude[PITCH].init(3.5, 0, 0,
						 0.001,
						 0,
						 0, 1,
						 1.0f / 1000);

	pid_attitude[YAW].init(3, 0, 0,
					   0.001,
					   0,
					   0, 1,
					   1.0f / 1000);
	
	for (;;)
	{
		float error, output;
		int32_t speed_1, speed_2;
		int32_t angle_1, angle_2;

		// if (imu.attitude_even())
		{
			// speed_1 = encoder_1.speedGet();
			// speed_2 = encoder_2.speedGet();

//			error = target_speed[0] - speed_1;

//			speed_ctrl_output[PITCH] = -pid_speed[PITCH].apply(error);


			vTaskDelayUntil(&tick_abs, 5);
		}
	}
}

void enc_control_task(void *argument)
{
	TickType_t tick_abs;
	volatile static float error;
	float rate[2];

	pid_enc[0].init(3.5, 0, 0,
					0.001,
					0,
					0, 1,
					1.0f / 1000);
	pid_enc[1].init(3.5, 0, 0,
					0.001,
					0,
					0, 1,
					1.0f / 1000);

	tick_abs = xTaskGetTickCount();

	for (;;)
	{

		// TODO: convert the rate of wheal
		rate[0] = target_speed - target_speed_angle;
		rate[1] = target_speed + target_speed_angle;

		for (size_t i = 0; i < 2; i++)
		{
			encoder[i].handle();

			speed[i] = encoder[i].speedGet();

			error = rate[i] - speed[i];			
			enc_ctrl_output[i] = pid_enc[i].apply(error);
		}

		vTaskDelayUntil(&tick_abs, 2);

		//DebugCnt++;
	}
}

// 20hz in max
void oled_task(void *argument)
{
	for (;;)
	{
		//oled_handle();
		vTaskDelay(1);
	}
}

void btn_task(void *argument)
{
	for (;;)
	{
		//button_handle();
		vTaskDelay(20);
	}
}

void sonic_task(void *argument)
{
	uint16_t distance;

	for (;;)
	{
		if (sonic.read(distance))
		{
		}

		mavlink_msg_nav_controller_output_send(MAVLINK_COMM_0, 0, 0, 0, 0, 0, distance, 9, 9);

		vTaskDelay(10);
	}
}

void battery_task(void *argument)
{
	uint32_t val;

	for (;;)
	{
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 10);
		if ((HAL_ADC_GetState(&hadc1) & HAL_ADC_STATE_EOC_REG) == HAL_ADC_STATE_EOC_REG)
		{
			val = HAL_ADC_GetValue(&hadc1);
			//sgw.battery = val * 4096;
		}

		vTaskDelay(10);
	}
}

void mavlink_send_bytes(mavlink_channel_t chan, const char *buf, uint16_t len)
{
	if (chan == MAVLINK_COMM_0)
	{
		memcpy(&mavlink_send_buffer[mavlink_send_pos], buf, len);
		mavlink_send_pos += len;
	}
}

void mavlink_send_cb(mavlink_channel_t chan, uint32_t len)
{
	if (chan == MAVLINK_COMM_0)
	{
		if (mavlink_send_pos == len)
		{
			if (mavlink != NULL)
				mavlink->write((uint8_t *)mavlink_send_buffer, len);
			mavlink_send_pos = 0;
		}
	}
}

void serial_task(void *argument)
{
	mavlink_message_t msg;
	mavlink_status_t status;
	uint8_t buffer[128];
	uint32_t size;
	TickType_t tick_abs;
	uint32_t count;

	tick_abs = xTaskGetTickCount();

	for (;;)
	{
		// period: 1000 ms
		size = 0;

		size = usb.read(buffer, 128);

		for (size_t i = 0; i < size; i++)
		{
			if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status))
			{
				mavlink_proc(msg);
			}
		}

		if (count++ > 100)
		{
			count = 0;
			mavlink_msg_heartbeat_send(MAVLINK_COMM_0, 3, 3, 1, 2, 0);
		}

		vTaskDelayUntil(&tick_abs, 10);
	}
}

static void mavlink_proc(mavlink_message_t &msg)
{
	switch (msg.msgid)
	{
	case MAVLINK_MSG_ID_HEARTBEAT:
	{
		mavlink_heartbeat_t heartbeat;
		mavlink_msg_heartbeat_decode(&msg, (mavlink_heartbeat_t *)&heartbeat);
		break;
	}
	default:
		vTaskDelay(1);
		break;
	}
}

void monitor_task(void *argument)
{
	uint8_t runtime_info[512];

	for (;;)
	{
//		shell->write((uint8_t *)"Tasks\t\tStatus\tPriority\tStack(4)\tNum\n\n");
//		vTaskList((char *)runtime_info);
//		shell->write(runtime_info);

//		for (size_t i = 0; i < 5; i++)
//			shell->write((uint8_t *)"\n");

//		shell->write((uint8_t *)"Tasks\t\tCPU(us)\t\tCPU(\%)\n\n");
//		vTaskGetRunTimeStats((char *)runtime_info);
//		shell->write((uint8_t *)runtime_info);

//		for (size_t i = 0; i < 5; i++)
//			shell->write((uint8_t *)"\n");

		vTaskDelay(1000);
	}
}

void periperal_task(void *argument)
{
	mpu_t sensor;
	TickType_t tick_abs;

	tick_abs = xTaskGetTickCount();

	for (;;)
	{
		mavlink_msg_raw_imu_send(MAVLINK_COMM_0, (uint64_t)tick_abs * 1000,
								 sensor.accel[0], sensor.accel[1], sensor.accel[2],
								 sensor.gyro[0], sensor.gyro[1], sensor.gyro[2],
								 sensor.magnet[0], sensor.magnet[1], sensor.magnet[2],
								 0,
								 sensor.temp);

		// led_red.handle();

		vTaskDelayUntil(&tick_abs, 1);
	}
}
