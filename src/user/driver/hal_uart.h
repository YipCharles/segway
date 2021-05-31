#pragma once

#include "datatypes.h"
#include "platform.h"

class DEV_uart
{
private:
	UART_HandleTypeDef *uart;

	SemaphoreHandle_t mutex;

	uint8_t *tx_buffer;
	uint8_t *rx_buffer;
	uint32_t buffer_size;

	// tx r/w is straight line
	uint32_t rx_write_cnt;
	uint32_t rx_read_cnt;

	bool ready;
public:
	DEV_uart();

	void init(UART_HandleTypeDef &h, uint32_t size);

	uint32_t read(uint8_t *data, uint32_t size, uint32_t time);
	uint32_t read(uint8_t *data, uint32_t size);

	uint32_t write(uint8_t *data, uint32_t size, uint32_t time);
	uint32_t write(uint8_t *data, uint32_t size);
	void write(uint8_t *s);

	void write_cb(UART_HandleTypeDef *huart);
};


