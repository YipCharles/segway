#include "platform.h"
#include "applications.h"
#include "oled.h"
#include "imu.h"
#include "mavlink.h"
#include "datatypes.h"
#include "LowPassFilter.h"
#include "LowPassFilter2p.h"

static TaskHandle_t handle_startup;
mavlink_system_t mavlink_system;
static uint8_t mavlink_send_buffer[256];
static uint32_t mavlink_send_pos = 0;
static DEV_uart *mavlink = &serial_1;
static DEV_usb *shell = &usb;
IMU imu;
extern MPU9250 mpu;

extern "C"
{
	extern void user_loop(void);
}

void test_task(void *argument);
void imu_task(void *argument);
void control_task(void *argument);
void oled_task(void *argument);
void periperal_task(void *argument);
void battery_task(void *argument);
void sonic_task(void *argument);
void serial_task(void *argument);
void monitor_task(void *argument);
void mavlink_proc(mavlink_message_t &msg);

void user_loop(void)
{
	xTaskCreate(startup_task, "Startup", 256, NULL, RTOS_PRIORITY_HIGH, &handle_startup);
	vTaskStartScheduler();
	while (1)
		;
}

void startup_task(void *argument)
{
	//	sgw_param_load_default();

	platform_init();

	mavlink_system.sysid = 0x21;
	mavlink_system.compid = 0;

	//	sgw_init();

	// task
	xTaskCreate(control_task, "Control", 256, NULL, RTOS_PRIORITY_HIGH, NULL);
	xTaskCreate(imu_task, "IMU", 1024, NULL, RTOS_PRIORITY_NORMAL, NULL);
	xTaskCreate(serial_task, "Serial", 256, NULL, RTOS_PRIORITY_NORMAL, NULL);
	xTaskCreate(sonic_task, "Sonic", 256, NULL, RTOS_PRIORITY_NORMAL, NULL);
	xTaskCreate(oled_task, "Oled", 256, NULL, RTOS_PRIORITY_LOWEST, NULL);
	xTaskCreate(battery_task, "Battery", 128, NULL, RTOS_PRIORITY_NORMAL, NULL);
	xTaskCreate(periperal_task, "Periperal", 256, NULL, RTOS_PRIORITY_NORMAL, NULL);
#ifdef DEBUG
	xTaskCreate(test_task, "Test", 1024, NULL, RTOS_PRIORITY_LOWEST, NULL);
	xTaskCreate(monitor_task, "Monitor", 256, NULL, RTOS_PRIORITY_NORMAL, NULL);
#endif

	vTaskDelete(handle_startup);
}

	float s, s_lp, s_lp_2;
void test_task(void *argument)
{
	float i = 0;
	TickType_t tick_abs;

	tick_abs = xTaskGetTickCount();
	
	LowPassFilterFloat lpf;
	LowPassFilter2pFloat lpf2;
	
	lpf.set_cutoff_frequency(1000, 1);
	lpf2.set_cutoff_frequency(1000, 10);
	
	for (;;)
	{
		s = sinf(i);
		if (s > 0)
			s=1;
		else
			s=-1;
		s_lp = lpf.apply(s);
		s_lp_2 = lpf2.apply(s);

		i+=0.1f;
		vTaskDelayUntil(&tick_abs, 1);
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

void imu_task(void *argument)
{
	imu.init();

	for (;;)
	{
		imu.handle();
	}
}

void control_task(void *argument)
{
	// float gyro_angle[3], accel_angle[3];
	// float torque[3];
	// float dt = 1.0f / 1000;
	// float error[3], error_int[3];
	// float kp = 0.0f, ki = 0.0f;
	// float output;
	TickType_t tick_abs;
	imu_sensor_t sensor;
	float euler_angle[3];

	tick_abs = xTaskGetTickCount();

	for (;;)
	{
		{
			//imu.read(&sensor, euler_angle);

		}
		
		vTaskDelayUntil(&tick_abs, 1);
	}
	// while (1)
	// {
	// 	imu_drv_handle(&sgw.sensor);

	// 	gyro_angle[ROLL] = sgw.imu.euler_angle[ROLL] + RAD_TO_DEG(sgw.sensor.gyro[ROLL]) * dt;
	// 	gyro_angle[PITCH] = sgw.imu.euler_angle[PITCH] + RAD_TO_DEG(sgw.sensor.gyro[PITCH]) * dt;
	// 	gyro_angle[YAW] = sgw.imu.euler_angle[YAW] + RAD_TO_DEG(sgw.sensor.gyro[YAW]) * dt;

	// 	accel_angle[ROLL] = RAD_TO_DEG(atan2f(-sgw.sensor.accel[PITCH], -sgw.sensor.accel[YAW]));
	// 	accel_angle[PITCH] = RAD_TO_DEG(atan2f(sgw.sensor.accel[ROLL], -sgw.sensor.accel[YAW]));

	// 	error[ROLL] = accel_angle[ROLL] - gyro_angle[ROLL];
	// 	error[PITCH] = accel_angle[PITCH] - gyro_angle[PITCH];

	// 	error_int[ROLL] += error[ROLL];
	// 	if (error_int[ROLL] > 10)
	// 		error_int[ROLL] = 10;
	// 	else if (error_int[ROLL] < -10)
	// 		error_int[ROLL] = -10;

	// 	error_int[PITCH] += error[PITCH];
	// 	if (error_int[PITCH] > 10)
	// 		error_int[PITCH] = 10;
	// 	else if (error_int[PITCH] < -10)
	// 		error_int[PITCH] = -10;

	// 	sgw.imu.euler_angle[ROLL] = gyro_angle[ROLL] + error[ROLL] * kp + error_int[ROLL] * ki;
	// 	sgw.imu.euler_angle[PITCH] = gyro_angle[PITCH] + error[PITCH] * kp + error_int[PITCH] * ki;
	// 	sgw.imu.euler_angle[YAW] = gyro_angle[YAW];

	// 	output = pid_regulator(0 - sgw.imu.euler_angle[PITCH], &sgw.pid_position);
	// 	torque[0] = torque[1] = pid_regulator(output - gyro_angle[PITCH], &sgw.pid_speed);

	// 	//hal_pwm_duty_write(torque[0], torque[1], 1.0f);
	//}
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

void mavlink_proc(mavlink_message_t &msg)
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
		shell->write((uint8_t *)"Tasks\t\tStatus\tPriority\tStack(4)\tNum\n\n");
		vTaskList((char *)runtime_info);
		shell->write(runtime_info);

		for (size_t i = 0; i < 5; i++)
			shell->write((uint8_t *)"\n");

		shell->write((uint8_t *)"Tasks\t\tCPU(us)\t\tCPU(\%)\n\n");
		vTaskGetRunTimeStats((char *)runtime_info);
		shell->write((uint8_t *)runtime_info);

		for (size_t i = 0; i < 5; i++)
			shell->write((uint8_t *)"\n");

		vTaskDelay(1000);
	}
}
