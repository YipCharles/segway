#include "platform.h"
#include "led.h"

LED::LED(void)
{
	pattern = 0;
	repeat = false;

	ready = true;
}

void LED::init(GPIO_TypeDef *p, uint16_t pi, bool l)
{
	port = p;
	pin=pi;
	level = l;

	pattern = 0;
	repeat = false;

	ready = true;
}

void LED::write(uint16_t p, bool r)
{
	if (!ready)
		return;

	pattern = p;
	repeat = r;
	pattern_temp = pattern;

	set(false);
}

void LED::write(bool en)
{
	if (!ready)
		return;

	pattern = 0;
	repeat = false;

	set(en);
}

void LED::set(bool en)
{
	if (en)
		HAL_GPIO_WritePin(port, pin, (GPIO_PinState)level);
	else
		HAL_GPIO_WritePin(port, pin, (GPIO_PinState)!level);
}

void LED::handle(void)
{
	if (!ready)
		return;

	if (pattern == 0)
		return;

	set(pattern_temp & 0x01);
	pattern_temp >>= 1;
	pattern_pos++;
	if (pattern_pos > 16 - 1)
	{
		pattern_temp = pattern;
		pattern_pos = 0;
	}
}

