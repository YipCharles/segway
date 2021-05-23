#include "imu.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "mpu9250.h"

IMU::IMU()
{
	// sensor_read = sensor_readIT = NULL;
	ready = false;
}

//IMU::IMU(MPU9250 &mpu)
//{
//	// sensor_init = &mpu.init;
//	// sensor_read = &mpu.read;
//	// sensor_readIT = mpu.request;

//	ready = false;
//}

extern MPU9250 mpu;

bool IMU::init(void)
{
	// sensor_read = mpu.read;
	// sensor_readIT = readIT;
	// sensor_read = mpu.read;

	exist = mpu.isready();
	sample_rate = 2000;

	// memcpy(&conf, addr, sizeof(imu_sensor_conf_t));

	conf.gyro_scale[0] = 500.0 / 32767 * 1.0f;
	conf.gyro_scale[1] = 500.0 / 32767 * 1.0f;
	conf.gyro_scale[2] = 500.0 / 32767 * 1.0f;

	conf.accel_scale[0] = 2.0 / 32767 * 1.0f;
	conf.accel_scale[1] = 2.0 / 32767 * 0.99f;
	conf.accel_scale[2] = 2.0 / 32767 * 1.0f;

	conf.gyro_offset[0] = -31;
	conf.gyro_offset[1] = 150;
	conf.gyro_offset[2] = -25;
	conf.accel_offset[0] = 0;
	conf.accel_offset[1] = 0;
	conf.accel_offset[2] = 0;
	conf.temp_offset = 3000;

	for (uint32_t i = 0; i < 3; i++)
	{
		gyro_lpf_handle[i].set_cutoff_frequency(sample_rate, 20);
		accel_lpf_handle[i].set_cutoff_frequency(sample_rate, 20);
	}

	imu_sample_queue = xQueueCreate(16, sizeof(mpu_t));

	return false;
}

void IMU::sample(void)
{
	if (!exist)
		return;

	mpu_t item;

	if (mpu.read(&item))
	{
		if (imu_sample_queue)
			xQueueSendFromISR(imu_sample_queue, &item, NULL);
	}

	// read_request can be NULL
	if (sensor_readIT)
		sensor_readIT();
}

bool IMU::handle(void)
{
	mpu_t item;
	bool result = false;

	if (!exist)
		return false;

	while (xQueueReceive(imu_sample_queue, &item, 0))
	{
#if 0
		// over scale
		for (uint32_t i = 0; i < 3; i++)
		{
			if (ABS(item.gyro[i]) > 32767 - 2)
				gyro_overflow = true;

			if (ABS(item.accel[i]) > 32767 - 2)
				accel_overflow = true;
		}
#endif

		// assemble problem
		gyro_raw[ROLL] = -item.gyro[1];
		gyro_raw[PITCH] = -item.gyro[0];
		gyro_raw[YAW] = item.gyro[2];
		accel_raw[ROLL] = -item.accel[1];
		accel_raw[PITCH] = -item.accel[0];
		accel_raw[YAW] = item.accel[2];

		/* calibrate */
		if (!calibrated)
		{
			if (1)
			{
				// clear the values delay in filter and queue
				// Queue_Clean(&icm_queue);

				for (uint32_t i = 0; i < 3; i++)
				{
					gyro_lpf_handle[i].reset();
					accel_lpf_handle[i].reset();
				}

				// MadgwickAHRS_Reset();

				calibrated = true;

				ready = true;
			}
		}
		else
		{
			// zero
			for (uint32_t i = 0; i < 3; i++)
			{
				gyro_zero[i] = (float)gyro_raw[i] - conf.gyro_offset[i];
				accel_zero[i] = (float)accel_raw[i] - conf.accel_offset[i];
			}

			// lpf first, for calibrate
			for (uint32_t i = 0; i < 3; i++)
			{
				if (gyro_lpf_handle[i].get_cutoff_freq())
					gyro_lpf[i] = gyro_lpf_handle[i].apply(gyro_zero[i]);

				if (accel_lpf_handle[i].get_cutoff_freq())
					accel_lpf[i] = accel_lpf_handle[i].apply(accel_zero[i]);
			}

			// convert to physic value
			// Temperature in Degrees Centigrade = (TEMP_DATA / 132.48) + 25
			for (uint32_t i = 0; i < 3; i++)
			{
				gyro[i] = gyro_lpf[i] * conf.gyro_scale[i];
				accel[i] = accel_lpf[i] * conf.accel_scale[i];
			}

#if 0			
			if (calibrate_update_need)
			{
				if (imu_calibrate_gyro_zero_update(s, true))
				{
					calibrate_update_need = false;
				}
			}
			
			// just for compare
			for (uint32_t i = 0; i < 3; i++)
			{
				angle[i] += gyro[i] / (float)IMU_SAMPLE_RATE;
			}
#endif
			// atittude estimation
			// MadgwickAHRS_Update(gyro[0], gyro[1], -gyro[2],
			// 					accel[0], accel[1], accel[2]);

			result = true;
		}
	}

	// TODO cant run MadgwickAHRS_Update

	if (0)
	{
		// very slow!
		// MadgwickAHRS_GetAngle(attitude);

	}

	return true;
}

bool IMU::is_ready(void)
{
	return ready;
}
