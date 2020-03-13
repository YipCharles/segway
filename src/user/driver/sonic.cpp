#include "platform.h"
#include "datatypes.h"
#include "sonic.h"

Sonic::Sonic(void)
{
	p_uart = NULL;
	ready = false;
}

void Sonic::init(DEV_uart &uart)
{
	p_uart = &uart;
	ready = true;
}

bool Sonic::read(uint16_t &d)
{
	if (!ready)
		return false;

	uint16_t distance = 0;
	uint8_t buffer[32];
	uint16_t size;

	size = p_uart->read(buffer, 32);

	for (uint32_t i = 0; i < size; i++)
	{
		if (parse(buffer[i], &distance))
		{
			d = distance;
			return false;
		}
	}

	return false;
}

bool Sonic::parse(uint8_t c, uint16_t *d)
{
	bool res;

	switch (status)
	{
	case 0:
		if (c == 0x5A)
		{
			status = 1;
			sum += c;
		}
		else
			status = 0;
		break;
	case 1:
		if (c == 0x5A)
		{
			status = 2;
			sum += c;
		}
		else
			status = 0;
		break;
	case 2:
		if (c == 0x45)
		{
			status = 3;
			sum += c;
		}
		else
			status = 0;
		break;
	case 3:
		if (c == 0x02)
		{
			status = 4;
			sum += c;
		}
		else
			status = 0;
		break;
	case 4:
		data = c << 8;
		sum += c;
		status = 5;
		break;
	case 5:
		data |= c;
		sum += c;
		status = 6;
		break;
	case 6:
		*d = data;
		if (sum == c)
			res = true;
		else
			res = false;

		status = 0;
		sum = 0;		
		break;
	default:
		break;
	}

	return res;
}
