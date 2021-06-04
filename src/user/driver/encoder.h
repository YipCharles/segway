#pragma once

#include "datatypes.h"
#include "LowPassFilter.h"
#include "LowPassFilter2p.h"
#include "FreeRTOS.h"
#include "queue.h"


class Encoder
{
private:
	uint32_t sample_rate;
	
	bool ready;

	int32_t edge_count;
	int64_t edge_count_all;

	float factor = 0.001;

	int32_t speed;		// mm/s
	int32_t distance;	// mm

	LowPassFilterInt lpf;

public:
	Encoder();

	// init(htime, channel);
	bool init(float f, uint32_t period);
	bool readyGet(void) { return ready; }
	
	void sample(bool pwm_pol);
	bool handle(void);

	int32_t speedGet(void) { return speed; }
	int32_t distanceGet(void) { return distance; }
};
