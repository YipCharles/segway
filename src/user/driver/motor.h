#pragma once

#include "datatypes.h"
#include "platform.h"

class Motor
{
private:
	TIM_HandleTypeDef *timer;
	uint32_t channel[2];

	bool ready;
	bool invert;

public:
	Motor();
	void init(TIM_HandleTypeDef *h, uint32_t ch1, uint32_t ch2, bool iv);
	bool write(float duty);
};

