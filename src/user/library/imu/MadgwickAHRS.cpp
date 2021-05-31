#include "MadgwickAHRS.h"
#include <math.h>

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

//=====================================================================================================
// IMU.c
// S.O.H. Madgwick
// 25th September 2010
//=====================================================================================================
// Description:
//
// Quaternion implementation of the 'DCM filter' [Mayhony et al].
//
// User must define 'halfT' as the (sample period / 2), and the filter gains 'Kp' and 'Ki'.
//
// Global variables 'q0', 'q1', 'q2', 'q3' are the quaternion elements representing the estimated
// orientation.  See my report for an overview of the use of quaternions in this application.
//
// User must call 'IMUupdate()' every sample period and parse calibrated gyroscope ('gx', 'gy', 'gz')
// and accelerometer ('ax', 'ay', 'ay') data.  Gyroscope units are radians/second, accelerometer
// units are irrelevant as the vector is normalised.
//
//=====================================================================================================

//----------------------------------------------------------------------------------------------------
// Header files

//----------------------------------------------------------------------------------------------------
// Definitions

#define IMU_SAMPLE_RATE 1000
#define halfT (0.5f / IMU_SAMPLE_RATE) // half the sample period

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile static float exInt = 0, eyInt = 0, ezInt = 0; // scaled integral error
volatile static float Kp, Ki, KpMax = 0, KiMax = 0;
volatile static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // quaternion of sensor frame relative to auxiliary frame
static float invSqrt(float x);
static uint32_t accel_stable_cnt;
static uint32_t accel_Calc_period;
static uint32_t accel_tick;

//====================================================================================================
// Function
//====================================================================================================
void MadgwickAHRS_Reset(void)
{
	q0 = 1;
	q1 = q2 = q3 = exInt = eyInt = ezInt = 0;
	Kp = Ki = 0;
	accel_stable_cnt = accel_tick = accel_Calc_period;
}

volatile static float acc_normalize;
volatile static bool need;
static uint32_t gyro_static_count = false;
volatile static float error = 0.001f;
float gravity_square = 0.999403119f;
static bool start = true;

void MadgwickAHRS_Update(float gx, float gy, float gz, float ax, float ay, float az)
{
	float norm;
	float vx, vy, vz;
	float ex, ey, ez;

	// death zone
	//	if (fabsf(gx) < 0.2 && fabsf(gy) < 0.2 && fabsf(gz) < 0.2)
	//	{
	//		if (gyro_static_count++ > IMU_SAMPLE_RATE*4)
	//			return;
	//	}
	//	else
	//		gyro_static_count = 0;

	// unit: rad
	gx = gx * (M_PI / 180.0);
	gy = gy * (M_PI / 180.0);
	gz = gz * (M_PI / 180.0);

	// gravity_square
	//	if (start)
	//	{
	//		start = false;
	//		gravity_square = ax * ax + ay * ay + az * az;
	//	}

	// normalise the measurements
	// but cost too much CPU
	accel_tick++;
	accel_stable_cnt++;
	// 10Hz
	if (accel_tick > (IMU_SAMPLE_RATE / 10))
	{
		accel_tick = 0;

		norm = invSqrt((ax * ax) + (ay * ay) + (az * az));
		acc_normalize = norm * 0.08 + acc_normalize * 0.92;

		if (fabsf(acc_normalize - gravity_square) > error)
		{
			accel_stable_cnt = 0;
			Kp = 0;
			Ki = 0;
		}
		else
		{
			// more than 1s
			if (accel_stable_cnt > IMU_SAMPLE_RATE)
			{
				// update Kp Ki
				// Kp = accel_stable_cnt * (KpMax/1000);
				// Ki = accel_stable_cnt * (KiMax/1000);
				// if (Kp > KpMax)
				// 	Kp = KpMax;
				// if (Ki > KiMax)
				// 	Ki = KiMax;
				Kp = KpMax;
				Ki = KiMax;

				// already normalize
				ax = ax * norm;
				ay = ay * norm;
				az = az * norm;

				// estimated direction of gravity
				// g vector in body frame
				vx = 2 * (q1 * q3 - q0 * q2);
				vy = 2 * (q0 * q1 + q2 * q3);
				vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

				// error is sum of cross product between reference direction of field and direction measured by sensor
				// cross product refer: https://onlinemschool.com/math/assistance/vector/multiply1/ (1,2,3 x 4,5,6)
				ex = (ay * vz - az * vy);
				ey = (az * vx - ax * vz);
				ez = (ax * vy - ay * vx);

				// integral error scaled integral gain
				exInt = exInt + ex * Ki;
				eyInt = eyInt + ey * Ki;
				ezInt = ezInt + ez * Ki;

				// adjusted gyroscope measurements
				gx = gx + Kp * ex + exInt;
				gy = gy + Kp * ey + eyInt;
				gz = gz + Kp * ez + ezInt;
			}
		}
	}

	// differential equation
	q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
	q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
	q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
	q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;

	// normalise quaternion
	norm = invSqrt((q0 * q0) + (q1 * q1) + (q2 * q2) + (q3 * q3));
	q0 = q0 * norm;
	q1 = q1 * norm;
	q2 = q2 * norm;
	q3 = q3 * norm;
}

//====================================================================================================
// END OF CODE
//====================================================================================================

// very slow!
void MadgwickAHRS_GetAngle(float angle[3])
{
	angle[0] = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * (180.0 / M_PI);
	angle[1] = asin(-2 * q1 * q3 + 2 * q0 * q2) * (180.0 / M_PI);
	angle[2] = -atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * (180.0 / M_PI);
}

// Carmack invert square
static float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long *)&y;
	i = 0x5f3759df - (i >> 1);
	y = *(float *)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
