#include "utils.h"

void angle_normalize_180(float *angle)
{
	while (*angle < -180.0f)
	{
		*angle += 360.0f;
	}
	while (*angle >= 180.0f)
	{
		*angle -= 360.0f;
	}
}

void angle_normalize_360(float *angle)
{
	while (*angle < 0.0f)
	{
		*angle += 360.0f;
	}
	while (*angle >= 360.0f)
	{
		*angle -= 360.0f;
	}
}

// -180~180
float angle_difference(float angle1, float angle2)
{
	float difference = angle1 - angle2;
	while (difference < -180.0f)
		difference += 2.0f * 180.0f;
	while (difference > 180.0f)
		difference -= 2.0f * 180.0f;
	return difference;
}

// intuitive way
float angles_mean(float *angles, int size)
{
	float mean;
	float *p_angle;
	float angle, angle0;

	p_angle = angles;
	angle0 = *angles;
	mean = 0;

	for (uint32_t i = 0; i < size; i++)
	{
		angle = *p_angle++;
		mean += angle0 + angle_difference(angle, angle0);
	}
	mean /= size;
	angle_normalize_360(&mean);

	return mean;
}

// use sin cos
float angles_mean2(float *angles, int size)
{
	float mean;
	float x, y;
	float rad;

	x = 0;
	y = 0;

	for (uint32_t i = 0; i < size; i++)
	{
		rad = DEG_TO_RAD(*angles);
		angles++;
		x += arm_cos_f32(rad);
		y += arm_sin_f32(rad);
	}

	mean = RAD_TO_DEG(atan2(y / size, x / size));
	angle_normalize_360(&mean);

	return mean;
}

// use complex num
float angles_mean3(float *angles, int size)
{
	float mean;
	return mean;
}

float angles_variance(float *angles, int size)
{
	float mean, variance, diff;
	float x, y;
	float rad;

	mean = angles_mean2(angles, size);
	
	x = 0;
	y = 0;

	for (uint32_t i = 0; i < size; i++)
	{
		diff = angle_difference(*angles, mean);
		angles++;
		rad = DEG_TO_RAD(diff);

		x += arm_cos_f32(rad * rad);
		y += arm_sin_f32(rad * rad);
	}
	variance = RAD_TO_DEG(atan2f(y, x));
	
	volatile static float a;
	a	= variance;
	
	return variance;
}

bool vector_2d_saturate(float *x, float *y, float max)
{
	bool retval = false;
	float mag = sqrtf(*x * *x + *y * *y);
	max = fabsf(max);

	if (mag < (float)(1e-10))
	{
		mag = 1e-10;
	}

	if (mag > max)
	{
		const float f = max / mag;
		*x *= f;
		*y *= f;
		retval = true;
	}

	return retval;
}


void lpf_poly_init(float *coef, uint32_t window_sz,float factor)
{
	float norm, point;

	norm = 0;
	
	for (int32_t i = 0; i < window_sz; i++)
	{
		point = powf((float)i, factor);		
		coef[window_sz - 1 - i] = point;
		norm += point;
	}

	if(norm > 0.001f)
	{
		for (size_t i = 0; i < window_sz; i++)
			coef[i] /= norm;
	}
}

float lpf_poly(float *window, float *coef, uint32_t window_sz, float sample)
{
	float v = 0;

	for (uint32_t i = window_sz - 1; i > 0; i--)
		window[i] = window[i - 1];
	
	window[0] = sample;
	
	for (size_t i = 0; i < window_sz; i++)
	{
		if (coef[i] > 0.001f)
			v += window[i] * coef[i];
	}
	
	return v;
}


