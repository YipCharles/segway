#pragma once

#include "datatypes.h"
#include "LowPassFilter2p.h"
#include "FreeRTOS.h"
#include "queue.h"

typedef enum
{
	ROLL = 0,
	PITCH,
	YAW,
} imu_axis_t;

typedef struct
{
	int16_t gyro[3];
	int16_t accel[3];
	int16_t temp;
	int16_t magnet[3];
} mpu_t;

typedef bool (*read_func_t)(mpu_t *sensor);
typedef void (*readIT_func_t)(void);

typedef struct
{
	int32_t gyro[3];
	int32_t accel[3];
	int32_t temp;
	int32_t magnet[3];
} icm_t;

typedef struct
{
	float gyro_scale[3];  // sensitivity
	float accel_scale[3]; // sensitivity * calibrate scale

	float gyro_offset[3]; // voliate
	float accel_offset[3];

	// float gyro_scale_calib[3];
	// float accel_scale_calib[3];

	float temp_offset;
	float temp_scale; // sensitivity * calibrate scale
} imu_sensor_conf_t;

// TODO: pack 32-bit align for better performance when pick up float
typedef struct
{
} imu_sensor_t;

class IMU
{
private:

	uint32_t sample_rate;
	bool exist;
	bool ready;

	bool calibrated; // first time calibrate the zero and know the noise value
	bool calibrate_update_need;
	bool calibrated_acc;

	bool gyro_overflow;
	bool accel_overflow;

	float noise_sq[3];

	read_func_t sensor_read;
	readIT_func_t sensor_readIT;

	// from sensor, assemble fix
	int16_t gyro_raw[3];
	int16_t accel_raw[3];
	int16_t temp_raw;

	// - zero
	float gyro_zero[3];
	float accel_zero[3];
	float temp_zero;

	// lpf
	float gyro_lpf[3];
	float accel_lpf[3];
	float temp_lpf;

	// physical value
	float gyro[3];
	float accel[3];
	float temp;

	LowPassFilter2pFloat gyro_lpf_handle[3];
	LowPassFilter2pFloat accel_lpf_handle[3];

	// single axis integral
	float angle[3];

	// quternion->euler
	// not NED, NEU for veheacle
	float attitude[3];

	// copy from flash
	imu_sensor_conf_t conf;

	// rtos
	QueueHandle_t imu_sample_queue = NULL;

public:
	IMU();

	bool init(void);
	bool init(imu_sensor_conf_t *addr);
	bool init(imu_sensor_conf_t *addr,
			  read_func_t read,
			  readIT_func_t readIT);
	bool is_ready(void);
	void sample(void);
	bool handle(void);
};


