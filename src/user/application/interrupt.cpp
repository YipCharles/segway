#include "platform.h"
#include "FreeRTOS.h"

extern IMU imu;
extern SemaphoreHandle_t sample_sync;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	serial_1.write_cb(huart);
}

extern "C"
{
//	void usb_read_cb(uint8_t *buffer, uint32_t size)
//	{
//		usb.read_cb(buffer, size);
//	}

//	void usb_write_cb(bool ok)
//	{
//		usb.write_cb(ok);
//	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM10)
	{
		HAL_IncTick();
		
	}

	else if (htim->Instance == TIM11)
	{
		if (sample_sync)
			xSemaphoreGiveFromISR(sample_sync, NULL);

	}
}
