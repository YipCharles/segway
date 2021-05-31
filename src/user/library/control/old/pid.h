#ifndef PID_H
#define PID_H
#include "datatypes.h"
#include "lpf_2p.h"

typedef struct
{
	float kp;
	float ki;
	float kd;

	float integral_limit;

	float output_limit;
	float output_factor;	// make sure params in near magnitude 

	lpf_2p_conf_t lpf;
} pid_param_t;

typedef struct
{
	pid_param_t *param;

	float error;
	float error_pre;
	float error_integral;
	float error_differential;
	float dt;
	float output;

	lpf_2p_t lpf;
} pid_t;

void pid_init(pid_t *pid, pid_param_t *pid_param, float dt);
float pid_regulator(float error, pid_t *pid);

#endif

