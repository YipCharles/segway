#include "platform.h"

// hal
DEV_uart serial_1;
DEV_uart serial_2;
DEV_usb usb;

// devices (refer from hal)
Sonic sonic;
Motor motor[2];
Encoder encoder[2];
MPU9250 mpu;
// LED led_red;
// Oled oled;

void platform_init(void)
{
	HAL_TIM_Base_Start_IT(&htim11);

	serial_1.init(huart1, 512);

	serial_2.init(huart2, 128);

	usb.init(512);

	mpu.init(&hspi1, SPI_CS_GPIO_Port, SPI_CS_Pin);

	sonic.init(serial_2);

	motor[0].init(&htim3, TIM_CHANNEL_1, TIM_CHANNEL_2, false);
	motor[1].init(&htim4, TIM_CHANNEL_1, TIM_CHANNEL_2, true);

	encoder[0].init(0.01, 500);
	encoder[1].init(0.01, 500);

	// led_red.init(LED_RED_GPIO_Port, LED_RED_Pin, false);
	// led_red.write(true);
	// led_red.write(false);

	oled_init();
	
}


