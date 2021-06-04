#pragma once

extern "C"
{
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
//#include "usbd_cdc_if.h"
}

#include "hal_uart.h"
//#include "hal_usb.h"

#include "sonic.h"
#include "motor.h"
#include "encoder.h"
#include "led.h"
#include "mpu9250.h"
#include "oled.h"

extern DEV_uart serial_1;
extern DEV_uart serial_2;
extern Sonic sonic;
extern LED led_red;
//extern Oled oled;
extern Motor motor_1, motor_2;
extern Encoder encoder_1, encoder_2;
//extern MPU9250 mpu;
//extern DEV_usb usb;

extern "C"
{
	void startup_task(void *argument);
	void oled_init(void);
	void oled_handle(void);
}

void platform_init(void);
