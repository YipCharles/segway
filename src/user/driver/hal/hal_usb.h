#pragma once

#include "datatypes.h"
#include "platform.h"

typedef void (*read_cb_t)(uint8_t *, uint32_t);

class DEV_usb
{
private:
	uint8_t *rx_buffer;
	uint8_t *tx_buffer;
	uint32_t buffer_size;

	uint32_t rx_size;
	uint32_t tx_size;

	SemaphoreHandle_t read_mutex;
	SemaphoreHandle_t write_mutex;

	bool usb_tx_ok;
	uint32_t timeout;
		
	bool ready;

public:
	DEV_usb();

	void init(uint32_t size);

	uint32_t read(uint8_t *buffer, uint32_t size);
	void write(uint8_t *buffer, uint32_t size);
	void write(uint8_t *s);

	void read_cb(uint8_t *buffer, uint32_t size);
	void write_cb(bool ok);
};
