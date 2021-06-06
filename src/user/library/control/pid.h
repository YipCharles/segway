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

	float _ierror_limit;
	float _out_limit;

	float _error;
	float _error_pre;
	float _ierror;
	float _derror;
	float _dt;
	float _output;

	LowPassFilterFloat lpf;

public:
	PID(){};

	void init(
		float kp,
		float ki,
		float kd,
		float ierror_limit,
		float out_limit,
		float d_cutoff,
		uint32_t run_rate);

	float apply(float error);
	void kpSet(float p);
	void kiSet(float i);
	void kdSet(float d);
	void resset(void);
};
