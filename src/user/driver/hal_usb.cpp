#include "datatypes.h"
#include "platform.h"

DEV_usb::DEV_usb()
{
	buffer_size = rx_size = tx_size = 0;
	rx_buffer = tx_buffer = NULL;

	ready = false;
}

void DEV_usb::init(uint32_t size)
{
	buffer_size = size;
	
	rx_buffer = (uint8_t *)pvPortMalloc(buffer_size);
	memset(rx_buffer, 0, buffer_size);
	// tx_buffer = (uint8_t *)pvPortMalloc(buffer_size);
	// memset(tx_buffer, 0, buffer_size);

	rx_size = tx_size = 0;

	read_mutex = xSemaphoreCreateBinary();
	write_mutex = xSemaphoreCreateBinary();

	xSemaphoreGive(read_mutex);
	xSemaphoreGive(write_mutex);

	ready = true;
}

uint32_t DEV_usb::read(uint8_t *buffer, uint32_t size)
{
	if (!ready)
		return 0;

	if (!rx_size)
		return 0;

	uint32_t once_size;

	//if (xSemaphoreTake(read_mutex, 10) == pdTRUE)
	{
		if (rx_size > size)
		{
			memcpy(buffer, rx_buffer, size);
			once_size = size;
			rx_size -= once_size;
			memcpy(buffer, buffer + once_size, rx_size);
		}
		else
		{
			memcpy(buffer, rx_buffer, rx_size);
			once_size = rx_size;
			rx_size = 0;
		}

		return once_size;
	}
	return 0;
}

void DEV_usb::write(uint8_t *buffer, uint32_t size)
{
	if (!ready)
		return;

	while (size)
	{
		// xSemaphoreTake(write_mutex, 100);
		if (xSemaphoreTake(write_mutex, 1) == pdTRUE)
		{
			timeout = 1;
			if (size > buffer_size)
			{
				while(CDC_Transmit_FS(buffer, buffer_size) != USBD_OK && timeout--)
					vTaskDelay(1);
				size -= buffer_size;
				buffer += buffer_size;
			}
			else
			{
				while(CDC_Transmit_FS(buffer, size) != USBD_OK && timeout--)
					vTaskDelay(1);
				size = 0;
			}
		}
	}
}

void DEV_usb::write(uint8_t *s)
{
	write(s, strlen((const char *)s));
}

void DEV_usb::read_cb(uint8_t *buffer, uint32_t size)
{
	if (!ready)
		return;

	// if (xSemaphoreTakeFromISR(read_mutex, NULL) == pdTRUE)
	{
		if (rx_size + size > buffer_size)
			size = buffer_size - rx_size;

		memcpy(rx_buffer + rx_size, buffer, size);
		rx_size += size;
	
//		xSemaphoreGiveFromISR(read_mutex, NULL);
	}
}

void DEV_usb::write_cb(bool ok)
{
	if (!ready)
		return;
	
	usb_tx_ok = ok;

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (usb_tx_ok)
	{
		xSemaphoreGiveFromISR(write_mutex, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

