#include "pid.h"

void PID::init(
	float kp,
	float ki,
	float kd,
	float out_factor,
	float cutoff_freq,
	float acc_limit,
	float out_limit,
	float dt)
{
	_kp = kp;
	_ki = ki;
	_kd = kd;
	
	_out_factor = out_factor;

	if (cutoff_freq)
		lpf.set_cutoff_frequency(cutoff_freq);

	_acc_limit = acc_limit;
	_out_limit = out_limit;

	_dt = dt;

	_error = 0;
	_error_pre = 0;
	_error_acc = 0;
	_error_diff = 0;
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
		_error_acc = 0;
	}
	else
	{
		_error_acc += _error;

		if (_error_acc > _acc_limit)
		{
			_error_acc = _acc_limit;
		}
		else if (_error_acc < -_acc_limit)
		{
			_error_acc = -_acc_limit;
		}
	}
	i = _ki * _error_acc;

	d = _kd * (_error - _error_pre) / _dt;

	if (lpf.get_cutoff_freq())
		d = lpf.apply(d);

	out = (p + i + d) * _out_factor;

	if (out > _out_limit)
		out = _out_limit;
	else if (out < -_out_limit)
		out = -_out_limit;

	_output = out;

	return _output;
}
