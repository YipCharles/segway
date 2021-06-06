#include "imu.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "mpu9250.h"
#include "MadgwickAHRS.h"

#define DEBUG_PREFIX volatile static

IMU::IMU()
{
	// sensor_read = sensor_read_it = NULL;
	ready = false;
}

//IMU::IMU(MPU9250 &mpu)
//{
//	// sensor_init = &mpu.init;
//	// sensor_read = &mpu.read;
//	// sensor_read_it = mpu.request;

//	ready = false;
//}

extern MPU9250 mpu;

bool IMU::init(void)
{
	// sensor_read = mpu.read;
	// sensor_read_it = readIT;
	// sensor_read = mpu.read;

	exist = mpu.isready();
	sample_rate = 1000;

	// memcpy(&conf, addr, sizeof(imu_sensor_conf_t));

	conf.gyro_scale[0] = 500.0 / 32767 * 1.0f;
	conf.gyro_scale[1] = 500.0 / 32767 * 1.0f;
	conf.gyro_scale[2] = 500.0 / 32767 * 1.0f;

	conf.accel_scale[0] = 2.0 / 32767 * 1.0f;
	conf.accel_scale[1] = 2.0 / 32767 * 1.0f;
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

	imu_sample_queue = xQueueCreate(32, sizeof(mpu_t));
	gyro_ready = xSemaphoreCreateBinary();
	attitude_ready = xSemaphoreCreateBinary();

	return false;
}

// 1kHz
void IMU::sample(void)
{
	if (!exist)
		return;

	mpu_t item;

	if (mpu.read(&item))
	{
		if (imu_sample_queue)
			xQueueSend(imu_sample_queue, &item, 1);
	}

	// read_request can be NULL
	if (sensor_read_it)
		sensor_read_it();
}

extern volatile int DebugCnt;

bool IMU::handle(void)
{
	mpu_t item;
	bool result = false;

	if (!exist)
		return false;

	if (reset)
	{
		reset = false;
		ready = false;

		xQueueReset(imu_sample_queue);

		// MadgwickAHRS_Reset();

		for (uint32_t i = 0; i < 3; i++)
		{
			gyro_lpf_handle[i].reset();
			accel_lpf_handle[i].reset();
		}
	}
	
	while (xQueueReceive(imu_sample_queue, &item, 1000))
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
		accel_raw[ROLL] = -item.accel[1]+1000;
		accel_raw[PITCH] = -item.accel[0];
		accel_raw[YAW] = item.accel[2];

		/* calibrate: maybe stuck here because of var compute */
		if (!calibrated_gyro)
		{
			if (calibrate_gyro())
			{
				calibrated_gyro = true;

				// if (calibrate_update_need)
				// TODO: angle changes with wrong speed in these time, try to compensate it

				// TODO: it should be all calibrated ready
				reset = true;
			}
		}
#if 1
		else if (!calibrated_acc && calibrated_gyro)
		{
			if (calibrate_accel())
			{
				calibrated_acc = true;
				reset = true;
			}
		}
#endif
		else
		{
			// - zero bias
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

			// tell ctrl thread to process
			if (gyro_ready)
				xSemaphoreGive(gyro_ready);

			// just for compare
			for (size_t i = 0; i < 3; i++)
				angle[i] += gyro[i] / (float)sample_rate;
	
			// o1: 100us
			// atittude estimation, sync with sample rate
			MadgwickAHRS_Update(gyro[0], gyro[1], -gyro[2],
								accel[0], accel[1], accel[2]);

			// tell ctrl thread to process
			// if (attitude_ready)
			// 	xSemaphoreGive(attitude_ready);

			// TODO: un-neccessary
			ready = true;

			DebugCnt++;
		}
	}

	return true;
}


//bool IMU::readyGet(void)
//{
//	return ready;
//}

bool IMU::calibrate_gyro(void)
{
	static uint32_t count = 0;
	static uint8_t axis = 0;
	static uint32_t stable_cnt = 0;
	float sq_diff = 0;
	uint32_t buffer_size = GYRO_CALIBRATE_SIZE * sizeof(uint16_t);
	int64_t sum = 0;
	DEBUG_PREFIX float variance, mean;

	if (count == 0 && axis == 0)
	{
		calibrate_buffer = (int16_t *)pvPortMalloc(buffer_size);
		buffer_indx = 0;
		memset(calibrate_buffer, 0, buffer_size);
		xQueueReset(imu_sample_queue);
	}

	calibrate_buffer[buffer_indx] = gyro_raw[axis];
	buffer_indx = (buffer_indx + 1) % GYRO_CALIBRATE_SIZE;

	if (count++ < GYRO_CALIBRATE_SIZE)
		return false;

	sum = 0;
	for (int i = 0; i < GYRO_CALIBRATE_SIZE; i++)
		sum += calibrate_buffer[i];
	mean = (float)sum / GYRO_CALIBRATE_SIZE;

	sq_diff = 0;
	for (int i = 0; i < GYRO_CALIBRATE_SIZE; i++)
	{
		float diff = (float)calibrate_buffer[i] - mean;
		sq_diff += diff * diff;
	}
	variance = sq_diff / GYRO_CALIBRATE_SIZE;

	if (variance < (calibrated_gyro ? gyro_noise_sq[axis] * 2 : 85))
	{
		if (stable_cnt++ > 100)
			; // stable_cnt=0;
		else
			return false;

		conf.gyro_offset[axis] = mean;
		gyro_noise_sq[axis] = variance;

		axis++;
		count = 0;
		buffer_indx = 0;
		memset(calibrate_buffer, 0, buffer_size);

		if (axis > 2)
		{
			axis = 0;

			vPortFree(calibrate_buffer);

			return true;
		}
	}
	else
		return false;

	return false;
}

bool IMU::calibrate_accel(void)
{
	static uint32_t count = 0;
	static uint8_t axis = 0;
	float sq_diff = 0;
	uint32_t buffer_size = GYRO_CALIBRATE_SIZE * sizeof(uint16_t);
	int64_t sum = 0;
	float variance, mean;

	if (count == 0 && axis == 0)
	{
		calibrate_buffer = (int16_t *)pvPortMalloc(buffer_size);
		buffer_indx = 0;
		memset(calibrate_buffer, 0, buffer_size);
		xQueueReset(imu_sample_queue);
	}

	calibrate_buffer[buffer_indx] = accel_raw[axis];
	buffer_indx = (buffer_indx + 1) % GYRO_CALIBRATE_SIZE;

	if (count++ < GYRO_CALIBRATE_SIZE)
		return false;

	sum = 0;
	for (int i = 0; i < GYRO_CALIBRATE_SIZE; i++)
		sum += calibrate_buffer[i];
	mean = (float)sum / GYRO_CALIBRATE_SIZE;

	sq_diff = 0;
	for (int i = 0; i < GYRO_CALIBRATE_SIZE; i++)
	{
		float diff = (float)calibrate_buffer[i] - mean;
		sq_diff += diff * diff;
	}
	variance = sq_diff / GYRO_CALIBRATE_SIZE;

	if (variance < (acc_noise_sq[axis] ? acc_noise_sq[axis] * 1.5 : 4000))
	{
		conf.accel_offset[axis] = mean;
		acc_noise_sq[axis] = variance;

		axis++;
		count = 0;
		buffer_indx = 0;
		memset(calibrate_buffer, 0, buffer_size);

		if (axis > 2)
		{
			axis = 0;

			vPortFree(calibrate_buffer);

			// z axis
			conf.accel_offset[YAW] = 0;
			conf.accel_scale[YAW] = 2.0 / 32767 * (16383 / mean);

			gravity_square = 0;

			return true;
		}
	}

	return false;
}

bool IMU::attitudeGet(float att[3])
{
	if (!ready)
		return false;
	
	MadgwickAHRS_GetAngle(attitude);
	
	memcpy(att, attitude, sizeof(attitude));

	return true;
}

bool IMU::gyroGet(float g[3])
{
	if (!ready)
		return false;

	memcpy(g, gyro, sizeof(gyro));

	return true;
}

bool IMU::accelGet(float a[3])
{
	if (!ready)
		return false;

	memcpy(a, accel, sizeof(accel));

	return true;
}

bool IMU::magGet(float m[3])
{
	if (!ready)
		return false;

	memcpy(m, mag, sizeof(mag));

	return true;
}

bool IMU::gyro_even()
{
	return (xSemaphoreTake(gyro_ready, 10) == pdTRUE);
}

bool IMU::attitude_even()
{
	return (xSemaphoreTake(attitude_ready, 10) == pdTRUE);
}


