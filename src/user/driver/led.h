#pragma once

#include "datatypes.h"
#include "platform.h"

class LED
{
private:
	GPIO_TypeDef *port;
	uint16_t pin;
	bool level;

	uint32_t pattern;
	uint32_t pattern_temp;

	// uint8_t pattern_count;
	uint8_t pattern_pos;
	bool repeat;

	bool ready;

	void set(bool en);

public:
	LED();
	void init(GPIO_TypeDef *po, uint16_t pi, bool l);
	void write(uint16_t p, bool r);
	void write(bool en);
	void handle(void);
};
