#pragma once

#include "datatypes.h"

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

typedef struct
{
	float gyro[3];
	float accel[3];
	float temp;
	float magnet[3];
} imu_sensor_t;

typedef struct
{
	float gyro_scale[3]; // sensitivity
	float gyro_offset[3];
	float accel_offset[3];
	float accel_scale[3]; // sensitivity * calibrate scale
	float temp_offset;
	float temp_scale; // sensitivity * calibrate scale
	float magnet_scale[3];
	float magnet_offset[3];
} sensor_conf_t;

typedef enum
{
	IMU_MODE_CALIBRATE = 0,
	IMU_MODE_RUN,

} imu_mode_t;

class IMU_driver
{
private:
	sensor_conf_t *cfg;

	imu_sensor_t sensor;
	float euler_angle[3];

public:
	IMU_driver();

	imu_mode_t mode;


	void init(sensor_conf_t *c);

	void handle(mpu_t *mpu);
	void read(imu_sensor_t *s, float angle[3]);

	void calibrate_gyro(mpu_t *mpu);
	void calibrate_accel(mpu_t *mpu);

	bool attitude_handle(void);
};
