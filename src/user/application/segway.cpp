#include "platform.h"
#include "segway.h"
#include "motor.h"
#include "mpu9250.h"
#include "utils.h"
#include "crc.h"

sgw_conf_t sgw_conf;
sgw_conf_t sgw_conf_default;
sgw_status_t sgw;


void sgw_init(void)
{
	hal_pwm_duty_write(0.0, 0.0, 1.0f);

	memset(&sgw, 0, sizeof sgw);

	sgw_conf.pid_position.kp = 2.5f;
	sgw_conf.pid_position.ki = 0.0f;
	sgw_conf.pid_position.kd = 0.0f;
	sgw_conf.pid_position.integral_limit = 10;
	sgw_conf.pid_position.output_limit = 500;
	sgw_conf.pid_position.output_factor = 0.05f;

	sgw_conf.pid_speed.kp = 0.5f;
	sgw_conf.pid_speed.ki = 0.0f;
	sgw_conf.pid_speed.kd = 0.0f;
	sgw_conf.pid_speed.integral_limit = 0.0f;
	sgw_conf.pid_speed.output_limit = 0.5f;
	sgw_conf.pid_speed.output_factor = 0.05f;
	
	pid_init(&sgw.pid_position, &sgw_conf.pid_position, 1.0f / 1000);
	pid_init(&sgw.pid_speed, &sgw_conf.pid_speed, 1.0f / 1000);

	sgw.sensor.init = hal_mpu9250_init;
	sgw.sensor.read = hal_mpu9250_read;

	imu_drv_init(&sgw.sensor, &sgw_conf.sensor);
	imu_drv_handle(&sgw.sensor);

}

void sgw_param_load_default(void)
{
	memset(&sgw_conf_default, 0, sizeof sgw_conf_default);

	sgw_conf_default.sensor.gyro_scale[0] = DEG_TO_RAD(500.0f / 32767);
	sgw_conf_default.sensor.gyro_scale[1] = DEG_TO_RAD(500.0f / 32767);
	sgw_conf_default.sensor.gyro_scale[2] = DEG_TO_RAD(500.0f / 32767);
	
	sgw_conf_default.sensor.accel_scale[0] = 8.0 / 32768;
	sgw_conf_default.sensor.accel_scale[1] = 8.0 / 32768;
	sgw_conf_default.sensor.accel_scale[2] = 8.0 / 32768;
	
	sgw_conf_default.pid_position.kp = 0.1;
	sgw_conf_default.pid_position.ki = 0.00;
	sgw_conf_default.pid_position.integral_limit = 200;
	sgw_conf_default.pid_position.output_limit = 1;
	
	sgw_conf_default.pid_speed.kp = 0.1;
	sgw_conf_default.pid_speed.ki = 0.00;
	sgw_conf_default.pid_speed.integral_limit = 200;
	sgw_conf_default.pid_speed.output_limit = 1;
	
	sgw_conf = sgw_conf_default;
}

// 1kHz task
void sgw_ctrl_process(void)
{
}

