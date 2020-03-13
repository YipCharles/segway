#include "datatypes.h"
#include "pid.h"

void pid_init(pid_t *pid, pid_param_t *pid_param, float dt)
{
	memset(pid, 0, sizeof(pid_t));

	pid->param = pid_param;
	pid->error = 0;
	pid->error_pre = 0;
	pid->error_integral = 0;
	pid->error_differential = 0;
	pid->output = 0;
	pid->dt = dt;

	if (pid_param->lpf.sample_rate && pid_param->lpf.cutoff_freq)
		lpf2_set_cutoff_frequency(&pid->lpf, pid_param->lpf.sample_rate, pid_param->lpf.cutoff_freq);
}

float pid_regulator(float error, pid_t *pid)
{
	float p_term, i_term, d_term, output, kp, ki, kd;
	float error_pre;

	kp = pid->param->kp;
	ki = pid->param->ki;
	kd = pid->param->kd;
	
	error_pre = pid->error;
	pid->error = error;
	
	p_term = kp * error;

	if (ki == 0)
	{
		pid->error_integral = 0;
	}
	else
	{
		pid->error_integral += error;
		
		if (pid->error_integral > pid->param->integral_limit)
		{
			pid->error_integral = pid->param->integral_limit;
		}
		else if (pid->error_integral < -pid->param->integral_limit)
		{
			pid->error_integral = -pid->param->integral_limit;
		}
	}
	i_term = ki * pid->error_integral;

	d_term = kd * (error - error_pre) / pid->dt;

	output = (p_term + i_term + d_term) * pid->param->output_factor;

	output = lpf2_apply(&pid->lpf, output);

	if (output > pid->param->output_limit)
		output = pid->param->output_limit;
	else if (output < -pid->param->output_limit)
		output = -pid->param->output_limit;

	pid->output = output;
	
	return pid->output;
}

