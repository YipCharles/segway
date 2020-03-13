#ifndef __MC_H
#define __MC_H
#include "datatypes.h"
#include "pid.h"
#include "imu.h"

typedef struct
{
	float position; // in 0~360°
	float speed;	// in dps
} target_t;

enum
{
	SGW_CTRL_IDLE = 0,
	SGW_CTRL_ENABLE = 1 << 0,
	SGW_CTRL_OVERRIDE = 1 << 1,
	SGW_CTRL_CURRENT = 1 << 2,
	SGW_CTRL_SPEED = 1 << 3,
	SGW_CTRL_POSITION = 1 << 4,
	SGW_CTRL_TEST = 1 << 5,
};

typedef enum
{
	SGW_MODE_IDLE = 0,
	SGW_MODE_CALIBRATE_AMP,
	SGW_MODE_CALIBRATE_ELEC,
	SGW_MODE_CALIBRATE_MECH,
	SGW_MODE_CALIBRATE_PID,
	SGW_MODE_RUN,
	SGW_MODE_STORE_SETTING,
} sgw_mode_t;

typedef struct
{
	sensor_conf_t sensor;

	pid_param_t pid_speed;
	pid_param_t pid_position;
} sgw_conf_t;

typedef struct
{
	sgw_mode_t mode;
	
	target_t target;

	sensor_t sensor;
	
	imu_status_t imu;
	
	float battery;
	uint32_t b;
	
	float duty_max;

	pid_t pid_position;
	pid_t pid_speed;

} sgw_status_t;

extern sgw_conf_t sgw_conf;
extern sgw_status_t sgw;

void sgw_init(void);
void sgw_param_load_default(void);

#endif



#pragma once

#include "datatypes.h"
#include "platform.h"

class Segway
{
private:
	sgw_mode_t mode;
	
	float battery;

public:
	Segway();



};
