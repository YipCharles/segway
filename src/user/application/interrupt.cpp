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

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_13)
	{
		encoder[0].sample(motor[0].readDuty() > 0);
	}
	
	else if (GPIO_Pin == GPIO_PIN_14)
	{
		encoder[1].sample(motor[1].readDuty() > 0);
	}
}
