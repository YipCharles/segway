#include "encoder.h"
#include "FreeRTOS.h"
#include "queue.h"

#define DEBUG_PREFIX volatile static

Encoder::Encoder()
{
	ready = false;
}

bool Encoder::init(float f, uint32_t period)
{
	lpf.set_cutoff_frequency(500, 5);

	factor = f * period;

	ready = true;

	return true;
}

void Encoder::sample(bool pwm_pol)
{
	if (ready)
	{
		if (pwm_pol)
			edge_count++;
		else
			edge_count--;

		edge_count_all += edge_count;

	}
}

// call this in accurate period
bool Encoder::handle(void)
{
	speed = lpf.apply(edge_count * factor);
	edge_count = 0;

	distance = edge_count_all * factor;

	return true;
}

