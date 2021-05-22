#include "platform.h"

// hal
DEV_uart serial_1;
DEV_uart serial_2;
DEV_usb usb;

// devices (refer from hal)
Sonic sonic;
Motor motor_1, motor_2;
MPU9250 mpu;
LED led_red;
// Oled oled;

void platform_init(void)
{
	// hal
	HAL_TIM_Base_Start(&htim2);

	// st already fix this bug
	// usb_init_malloc = (uint8_t *)pvPortMalloc(sizeof(USBD_CDC_HandleTypeDef));

	serial_1.init(huart1, 512);

	serial_2.init(huart2, 128);

	usb.init(512);

	// devices
	mpu.init(SPI1_CS_1_GPIO_Port, SPI1_CS_1_Pin);

	sonic.init(serial_2);

	motor_1.init(&htim3, TIM_CHANNEL_1, TIM_CHANNEL_2);
	motor_2.init(&htim4, TIM_CHANNEL_1, TIM_CHANNEL_2);

	// led_red.init(LED_RED_GPIO_Port, LED_RED_Pin, false);
	// led_red.write(true);
	// led_red.write(false);

	oled_init();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	serial_1.write_cb(huart);
}

extern "C"
{

	void usb_read_cb(uint8_t *buffer, uint32_t size)
	{
		usb.read_cb(buffer, size);
	}

	void usb_write_cb(bool ok)
	{
		usb.write_cb(ok);
	}
}