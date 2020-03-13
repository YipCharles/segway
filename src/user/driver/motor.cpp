#include "platform.h"
#include "motor.h"


Motor::Motor(void)
{
	ready = false;
}

void Motor::init(TIM_HandleTypeDef *h, uint32_t ch1,uint32_t ch2)
{
	timer = h;

	channel[0] = ch1;
	channel[1] = ch2;

	__HAL_TIM_SET_COMPARE(timer, channel[0], 0);
	__HAL_TIM_SET_COMPARE(timer, channel[1], 0);

	HAL_TIM_PWM_Start(timer, channel[0]);
	HAL_TIM_PWM_Start(timer, channel[1]);

	ready = true;
}

bool Motor::write(float duty)
{
	if (!ready)
		return false;
	
	uint32_t arr = __HAL_TIM_GetAutoreload(timer);

	if (duty > 0)
	{
		__HAL_TIM_SET_COMPARE(timer, channel[0], duty * arr);
		__HAL_TIM_SET_COMPARE(timer, channel[1], 0);
	}
	else
	{
		__HAL_TIM_SET_COMPARE(timer, channel[0], 0);
		__HAL_TIM_SET_COMPARE(timer, channel[1], -duty * arr);
	}

	return true;
}




