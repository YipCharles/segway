#ifndef _UTILS_H
#define _UTILS_H
#include "datatypes.h"

#define LPF_FAST(_value, _sample, _factor) \
									(_value -= (_factor) * ((_value) - (_sample)))
#define ELEMENT_OF(__BUFFER) 		(sizeof(__BUFFER) / sizeof(*(__BUFFER)))
#define OFFSET_ADDR(type, ele) 		((unsigned int)(&((type *)0)->ele))
#define DEG_TO_RAD(angleDegrees) 	((angleDegrees)*M_PI / 180.0f)
#define RAD_TO_DEG(angleRadians) 	((angleRadians)*180.0f / M_PI)


void angle_normalize_180(float *angle);
void angle_normalize_360(float *angle);
float angle_difference(float angle1, float angle2);
float angles_mean(float *angles, int angles_num);
float angles_mean2(float *angles, int angles_num);
float angles_variance(float *angles, int angles_num);
bool vector_2d_saturate(float *x, float *y, float max);
void lpf_poly_init(float *coef, uint32_t window_sz, float factor);
float lpf_poly(float *window, float *coef, uint32_t window_sz, float sample);

#endif
