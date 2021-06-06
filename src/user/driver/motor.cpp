#include "platform.h"
#include "motor.h"

// have to raise up slowly
//	while(1)
//	{
//		static float i=0;
//			motor_1.write(i+=0.01);
//			motor_2.write(i+=0.01);
//		vTaskDelay(10);	
//	}


Motor::Motor(void)
{
	ready = false;
}

void Motor::init(TIM_HandleTypeDef *h, uint32_t ch1, uint32_t ch2, bool iv)
{
	timer = h;
	
	invert = iv;

	channel[0] = ch1;
	channel[1] = ch2;

	__HAL_TIM_SET_COMPARE(timer, channel[0], 0);
	__HAL_TIM_SET_COMPARE(timer, channel[1], 0);

	HAL_TIM_PWM_Start(timer, channel[0]);
	HAL_TIM_PWM_Start(timer, channel[1]);

	write(0);
	
	ready = true;
}

bool Motor::write(float d)
{
	if (!ready)
		return false;

	duty = d;

	if (invert)
		d = -d / 1000.0;
	else
		d = d / 1000.0;

	uint32_t arr = __HAL_TIM_GetAutoreload(timer);

	if (d > 0)
	{
		__HAL_TIM_SET_COMPARE(timer, channel[0], d * arr);
		__HAL_TIM_SET_COMPARE(timer, channel[1], 0);
	}
	else
	{
		__HAL_TIM_SET_COMPARE(timer, channel[0], 0);
		__HAL_TIM_SET_COMPARE(timer, channel[1], -d * arr);
	}

	return true;
}




