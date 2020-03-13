::@echo off & cls & color 0b
cd /d %~dp0
::xcopy /y /r fix\ARM_CM4F		Middlewares\Third_Party\FreeRTOS\Source\portable\RVDS\ARM_CM4F
::xcopy /y /r fix\CMSIS_RTOS_V2		Middlewares\Third_Party\FreeRTOS\Source\CMSIS_RTOS_V2
xcopy /y /r fix\usb\Core		Middlewares\ST\STM32_USB_Device_Library\Core\Inc
xcopy /y /r fix\usb\CDC		Middlewares\ST\STM32_USB_Device_Library\Class\CDC
