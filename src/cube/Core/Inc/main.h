/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BTN_1_Pin GPIO_PIN_13
#define BTN_1_GPIO_Port GPIOC
#define BTN_2_Pin GPIO_PIN_14
#define BTN_2_GPIO_Port GPIOC
#define BTN_3_Pin GPIO_PIN_15
#define BTN_3_GPIO_Port GPIOC
#define BTN_4_Pin GPIO_PIN_0
#define BTN_4_GPIO_Port GPIOA
#define BTN_5_Pin GPIO_PIN_1
#define BTN_5_GPIO_Port GPIOA
#define ADC_BATT_Pin GPIO_PIN_4
#define ADC_BATT_GPIO_Port GPIOA
#define SPI1_CS_1_Pin GPIO_PIN_0
#define SPI1_CS_1_GPIO_Port GPIOB
#define SPI1_CS_2_Pin GPIO_PIN_1
#define SPI1_CS_2_GPIO_Port GPIOB
#define ENC_A_Pin GPIO_PIN_13
#define ENC_A_GPIO_Port GPIOB
#define ENC_A_EXTI_IRQn EXTI15_10_IRQn
#define ENC_B_Pin GPIO_PIN_14
#define ENC_B_GPIO_Port GPIOB
#define ENC_B_EXTI_IRQn EXTI15_10_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
