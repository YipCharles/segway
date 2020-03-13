#pragma once

#include "datatypes.h"
#include "platform.h"

class Sonic
{
private:
	// bus
	DEV_uart *p_uart;
	// parse
	uint8_t status = 0;
	uint16_t data = 0;
	uint8_t sum = 0;

	bool parse(uint8_t c, uint16_t *d);
	bool ready;
public:
	Sonic(void);
	void init(DEV_uart &uart);
	bool read(uint16_t &d);
};



