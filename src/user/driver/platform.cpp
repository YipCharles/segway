#include "platform.h"

// hal
DEV_uart serial_1;
DEV_uart serial_2;
DEV_usb usb;

// devices (refer from hal)
Sonic sonic;
Motor motor_1, motor_2;
MPU9250 mpu(&hspi1, SPI1_CS_1_GPIO_Port, SPI1_CS_1_Pin);
LED led_red;
// Oled oled;

void platform_init(void)
{
	HAL_TIM_Base_Start_IT(&htim2);

	serial_1.init(huart1, 512);

	serial_2.init(huart2, 128);

	usb.init(512);

	mpu.init();

	sonic.init(serial_2);

	motor_1.init(&htim3, TIM_CHANNEL_1, TIM_CHANNEL_2);
	motor_2.init(&htim4, TIM_CHANNEL_1, TIM_CHANNEL_2);

	// led_red.init(LED_RED_GPIO_Port, LED_RED_Pin, false);
	// led_red.write(true);
	// led_red.write(false);

	oled_init();
}


