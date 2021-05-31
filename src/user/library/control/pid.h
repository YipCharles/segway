#pragma once

#include "datatypes.h"
#include "LowPassFilter.h"
#include "LowPassFilter2p.h"

class PID
{
private:
	float _kp;
	float _ki;
	float _kd;

	float _out_factor;
	float _acc_limit;
	float _out_limit;

	float _error;
	float _error_pre;
	float _error_acc;
	float _error_diff;
	float _dt;
	float _output;

	LowPassFilterFloat lpf;

public:
	PID(){};

	void init(float kp,
			  float ki,
			  float kd,
			  float out_factor,
			  float cutoff_freq,
			  float acc_limit,
			  float out_limit,
			  float dt);

	float apply(float error);
	void kpSet(float p);
	void kiSet(float i);
	void kdSet(float d);
	void resset(void);
};
