#include "platform.h"
#include "imu.h"

IMU_driver::IMU_driver()
{
	
}

void IMU_driver::init(sensor_conf_t *c)
{
	cfg = c;

	// lpf2_set_cutoff_frequency(&s->lpf_handle, 1000, 40);

	// sample rate: 8khz
	// output rate: 8khz
	//dps / 32767
	// s->gyro_scale[0] = 500.0f / 32767;
	// s->gyro_scale[1] = 500.0f / 32767;
	// s->gyro_scale[2] = 500.0f / 32767;

	// sample rate: 1khz
	// output rate: 8khz
	// have to x 4/32768 firstly, but fusion with the scale.
	// s->accel_scale[0] = 8.0 / 32768;
	// s->accel_scale[1] = 8.0 / 32768;
	// s->accel_scale[2] = 8.0 / 32768;
}

void IMU_driver::handle(mpu_t *mpu)
{
	mpu_t mpu_2;

	// assemble problem
	mpu_2.gyro[ROLL] = -mpu->gyro[1];
	mpu_2.gyro[PITCH] = -mpu->gyro[0];
	mpu_2.gyro[YAW] = -mpu->gyro[2];
	mpu_2.accel[ROLL] = -mpu->accel[1];
	mpu_2.accel[PITCH] = -mpu->accel[0];
	mpu_2.accel[YAW] = -mpu->accel[2];
	mpu_2.temp = mpu->temp;

	// convert to physic value
	for (uint8_t i = 0; i < 3; i++)
	{
		sensor.gyro[i] = ((float)mpu_2.gyro[i] - cfg->gyro_offset[i]) * cfg->gyro_scale[i];
		sensor.accel[i] = ((float)mpu_2.accel[i] - cfg->accel_offset[i]) * cfg->accel_scale[i];
	}

	sensor.temp = ((float)mpu_2.temp- cfg->temp_offset) * cfg->temp_scale;

	// if (s->lpf_handle._cutoff_freq)
	// {
	// 	for (uint32_t i = 0; i < 3; i++)
	// 	{
	// 		s->gyro_lpf[i] = lpf2_apply(&s->lpf_handle, s->gyro[i]);
	// 		s->accel_lpf[i] = lpf2_apply(&s->lpf_handle, s->accel[i]);
	// 	}

	// 	s->temp_lpf = lpf2_apply(&s->lpf_handle, s->temp);
	// }
}

void IMU_driver::calibrate_gyro(mpu_t *mpu)
{
	uint32_t cnt;
	float avg[3];

	cnt = 200;
	for (size_t i = 0; i < 3; i++)
	{
		cfg->gyro_offset[i] = 0;
		avg[i] = 0;
	}

	while (cnt-- > 0)
	{
		for (size_t i = 0; i < 3; i++)
			avg[i] += mpu->gyro[i] / cfg->gyro_scale[i];

		vTaskDelay(1);
	}

	for (size_t i = 0; i < 3; i++)
		cfg->gyro_offset[i] = avg[i] / 200;
}
