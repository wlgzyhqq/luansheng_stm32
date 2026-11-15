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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
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
#define LED_Pin GPIO_PIN_5
#define LED_GPIO_Port GPIOE
#define Motor3_IN1_Pin GPIO_PIN_4
#define Motor3_IN1_GPIO_Port GPIOF
#define Stepper1_DIR_Pin GPIO_PIN_5
#define Stepper1_DIR_GPIO_Port GPIOF
#define Relay4_Pin GPIO_PIN_3
#define Relay4_GPIO_Port GPIOC
#define Stepper3_DIR_Pin GPIO_PIN_4
#define Stepper3_DIR_GPIO_Port GPIOC
#define Motor1_IN1_Pin GPIO_PIN_7
#define Motor1_IN1_GPIO_Port GPIOE
#define Motor2_IN2_Pin GPIO_PIN_8
#define Motor2_IN2_GPIO_Port GPIOE
#define Motor2_IN1_Pin GPIO_PIN_15
#define Motor2_IN1_GPIO_Port GPIOE
#define Stepper1_STEP_Pin GPIO_PIN_12
#define Stepper1_STEP_GPIO_Port GPIOD
#define Motor3_IN2_Pin GPIO_PIN_13
#define Motor3_IN2_GPIO_Port GPIOD
#define Stepper2_DIR_Pin GPIO_PIN_7
#define Stepper2_DIR_GPIO_Port GPIOG
#define Relay3_Pin GPIO_PIN_15
#define Relay3_GPIO_Port GPIOA
#define Relay2_Pin GPIO_PIN_9
#define Relay2_GPIO_Port GPIOG
#define Relay1_Pin GPIO_PIN_8
#define Relay1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
