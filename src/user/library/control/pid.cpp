#include "pid.h"

void PID::init(
	float kp,
	float ki,
	float kd,
	float ierror_limit,
	float out_limit,
	float d_cutoff,
	uint32_t run_rate)
{
	_kp = kp;
	_ki = ki;
	_kd = kd;

	if (d_cutoff)
		lpf.set_cutoff_frequency(run_rate, d_cutoff);

	_ierror_limit = ierror_limit;
	_out_limit = out_limit;

	_dt = 1.0f / run_rate;

	_error = 0;
	_error_pre = 0;
	_ierror = 0;
	_derror = 0;
	_output = 0;
}

float PID::apply(float error)
{
	float p, i, d, out;

	_error_pre = _error;
	_error = error;

	p = _kp * _error;

	if (is_zero(_ki))
	{
		_ierror = 0;
	}
	else
	{
		_ierror += _error;

		if (_ierror > _ierror_limit)
		{
			_ierror = _ierror_limit;
		}
		else if (_ierror < -_ierror_limit)
		{
			_ierror = -_ierror_limit;
		}
	}
	i = _ki * _ierror;

	d = (_error - _error_pre) / _dt;

	if (lpf.get_cutoff_freq())
		d = lpf.apply(d);
	
	// debug
	_derror = d;
	
	d = _kd * _derror;
	
	out = (p + i + d);

	if (out > _out_limit)
		out = _out_limit;
	else if (out < -_out_limit)
		out = -_out_limit;

	_output = out;

	return _output;
}
