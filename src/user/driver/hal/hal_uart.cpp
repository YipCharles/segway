#include "datatypes.h"
#include "platform.h"

/*
	@note: Test code (HAL lib has some problem in tx rx in the same time!)
	if(hal_uart_read(&huart_debug, &msg, portMAX_DELAY))
		hal_uart_write(&huart_debug, &msg, portMAX_DELAY);
*/

DEV_uart::DEV_uart()
{
	tx_buffer = rx_buffer =  NULL;
	mutex = NULL;
	
	buffer_size = rx_write_cnt = rx_read_cnt = 0;

	ready = false;
}

// uint32_t baudrate
void DEV_uart::init(UART_HandleTypeDef &h, uint32_t size)
{
	uart = &h;
	buffer_size = size;

	tx_buffer = (uint8_t *)pvPortMalloc(buffer_size);
	memset(tx_buffer, 0, buffer_size);

	rx_buffer = (uint8_t *)pvPortMalloc(buffer_size);
	memset(rx_buffer, 0, buffer_size);
	rx_read_cnt = rx_write_cnt = 0;

	mutex = xSemaphoreCreateBinary();
	xSemaphoreGive(mutex);

	HAL_UART_Receive_DMA(uart, rx_buffer, buffer_size);

	ready = true;
}

uint32_t DEV_uart::write(uint8_t *data, uint32_t size, uint32_t time)
{
	if (!ready)
		return false;
	if (size == 0)
		return false;
	

	bool done = false;

	if (xSemaphoreTake(mutex, time) == pdTRUE)
	{
		if (size > buffer_size)
			size = buffer_size;

		memcpy(tx_buffer, data, size);
		HAL_UART_Transmit_DMA(uart, tx_buffer, size);
		done = true;
		
	}

	return done;
}

uint32_t DEV_uart::write(uint8_t *data, uint32_t size)
{
	if (!ready)
		return false;
	if (size == 0)
		return false;
	

	bool done = false;

	//if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE)
	{
		if (size > buffer_size)
			size = buffer_size;

		memcpy(tx_buffer, data, size);
		//HAL_UART_Transmit_DMA(uart, tx_buffer, size);
		HAL_UART_Transmit(uart, tx_buffer, size, 10);
		done = true;
		
	}

	return done;
}

void DEV_uart::write(uint8_t *s)
{
	write(s, strlen((const char *)s));
}

// call by HAL_UART_TxCpltCallback
// But there is bug when release mutex in irq.
void DEV_uart::write_cb(UART_HandleTypeDef *h)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if(uart == h)
	{
		xSemaphoreGiveFromISR(mutex, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

uint32_t DEV_uart::read(uint8_t *data, uint32_t size, uint32_t timeout)
{
	if (!ready)
		return 0;

	uint32_t dma_cnt;
	uint32_t rx_data_cnt;

	rx_data_cnt = 0;
	dma_cnt = __HAL_DMA_GET_COUNTER(uart->hdmarx);
	rx_write_cnt = buffer_size - dma_cnt;

	while (1)
	{
		while (rx_read_cnt != rx_write_cnt)
		{
			data[rx_data_cnt] = rx_buffer[rx_read_cnt];

			rx_read_cnt++;
			if (rx_read_cnt >= buffer_size)
				rx_read_cnt = 0;

			rx_data_cnt++;
			if (rx_data_cnt >= size)
				break;
		}

		if (rx_data_cnt)
		{
			break;
		}
		// time out check
		else
		{
			if (timeout-- > 0)
				vTaskDelay(1);
			else
				break;
		}
	}

	return rx_data_cnt;
}

uint32_t DEV_uart::read(uint8_t *data, uint32_t size)
{
	if (!ready)
		return 0;

	return read(data, size, 0);
}


