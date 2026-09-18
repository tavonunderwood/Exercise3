/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32u5xx_hal.h"

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
#define USER_BUTTON_Pin GPIO_PIN_13
#define USER_BUTTON_GPIO_Port GPIOC
#define A4_Pin GPIO_PIN_1
#define A4_GPIO_Port GPIOC
#define A1_Pin GPIO_PIN_3
#define A1_GPIO_Port GPIOC
#define AS_Pin GPIO_PIN_0
#define AS_GPIO_Port GPIOA
#define AS_EXTI_IRQn EXTI0_IRQn
#define BS_Pin GPIO_PIN_1
#define BS_GPIO_Port GPIOA
#define BS_EXTI_IRQn EXTI1_IRQn
#define A2_Pin GPIO_PIN_2
#define A2_GPIO_Port GPIOA
#define A3_Pin GPIO_PIN_0
#define A3_GPIO_Port GPIOB
#define D8_Pin GPIO_PIN_12
#define D8_GPIO_Port GPIOF
#define D8_EXTI_IRQn EXTI12_IRQn
#define D4_Pin GPIO_PIN_14
#define D4_GPIO_Port GPIOF
#define D2_Pin GPIO_PIN_15
#define D2_GPIO_Port GPIOF
#define D5_Pin GPIO_PIN_11
#define D5_GPIO_Port GPIOE
#define D3_Pin GPIO_PIN_13
#define D3_GPIO_Port GPIOE
#define D10_Pin GPIO_PIN_14
#define D10_GPIO_Port GPIOD
#define D9_Pin GPIO_PIN_15
#define D9_GPIO_Port GPIOD
#define LED_RED_Pin GPIO_PIN_2
#define LED_RED_GPIO_Port GPIOG
#define D1_Pin GPIO_PIN_7
#define D1_GPIO_Port GPIOG
#define D0_Pin GPIO_PIN_8
#define D0_GPIO_Port GPIOG
#define LED_GREEN_Pin GPIO_PIN_7
#define LED_GREEN_GPIO_Port GPIOC
#define LED_BLUE_Pin GPIO_PIN_7
#define LED_BLUE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
