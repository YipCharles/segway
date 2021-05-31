#pragma once

#include "datatypes.h"
#include "LowPassFilter2p.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

#define GYRO_CALIBRATE_SIZE	128

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
typedef void (*readit_func_t)(void);

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
	bool reset;

	bool calibrated_gyro; // first time calibrate the zero and know the noise value
	bool calibrate_update_need;
	bool calibrated_acc;

	bool gyro_overflow;
	bool accel_overflow;

	float gyro_noise_sq[3];
	float acc_noise_sq[3];
	float gravity_square;

	bool (*sensor_init)(void);
	bool (*sensor_read)(int16_t gyro[3], int16_t accel[3], int16_t temp[1]);
	void (*sensor_read_it)(void);

	// from sensor, assemble fix
	int16_t gyro_raw[3];
	int16_t accel_raw[3];
	int16_t mag_raw[3];
	int16_t temp_raw;

	// - zero
	float gyro_zero[3];
	float accel_zero[3];
	float mag_zero[3];
	float temp_zero;

	// lpf
	float gyro_lpf[3];
	float accel_lpf[3];
	float mag_lpf[3];
	float temp_lpf;

	// physical value
	float gyro[3];
	float accel[3];
	float mag[3];
	float temp;

	// single axis integral
	float angle[3];
	// quternion->euler
	// not NED, NEU for veheacle
	float attitude[3];
	float yaw_accumulate;

	// lpf
	LowPassFilter2pFloat gyro_lpf_handle[3];
	LowPassFilter2pFloat accel_lpf_handle[3];
	LowPassFilter2pFloat mag_lpf_handle[3];

	// copy from flash
	imu_sensor_conf_t conf;

	// rtos
	QueueHandle_t imu_sample_queue = NULL;
	SemaphoreHandle_t gyro_ready = NULL;
	SemaphoreHandle_t attitude_ready = NULL;

	int16_t *calibrate_buffer = NULL;
	uint32_t buffer_indx = 0;

public:
	IMU();

	bool init(void);
	bool init(imu_sensor_conf_t *addr);
	bool init(imu_sensor_conf_t *addr,
			  read_func_t read,
			  readit_func_t readIT);
	bool readyGet(void) { return ready; }
	void resetSet(void) { reset = true; }
	void sample(void);
	bool handle(void);
	bool calibrate_gyro(void);
	bool calibrate_accel(void);
	bool attitudeGet(float att[3]);
	bool gyroGet(float g[3]);
	bool gyro_even();
	bool attitude_even();
	;
};
