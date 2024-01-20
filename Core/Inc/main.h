/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#define LED2_Pin GPIO_PIN_0
#define LED2_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_1
#define LED1_GPIO_Port GPIOC
#define U1_Pin GPIO_PIN_0
#define U1_GPIO_Port GPIOA
#define U2_Pin GPIO_PIN_1
#define U2_GPIO_Port GPIOA
#define U3_Pin GPIO_PIN_2
#define U3_GPIO_Port GPIOA
#define U4_Pin GPIO_PIN_3
#define U4_GPIO_Port GPIOA
#define U5_Pin GPIO_PIN_4
#define U5_GPIO_Port GPIOA
#define U6_Pin GPIO_PIN_5
#define U6_GPIO_Port GPIOA
#define U7_Pin GPIO_PIN_6
#define U7_GPIO_Port GPIOA
#define U8_Pin GPIO_PIN_7
#define U8_GPIO_Port GPIOA
#define U14_Pin GPIO_PIN_10
#define U14_GPIO_Port GPIOC
#define U15_Pin GPIO_PIN_11
#define U15_GPIO_Port GPIOC
#define U16_Pin GPIO_PIN_12
#define U16_GPIO_Port GPIOC
#define U9_Pin GPIO_PIN_3
#define U9_GPIO_Port GPIOB
#define U10_Pin GPIO_PIN_4
#define U10_GPIO_Port GPIOB
#define U11_Pin GPIO_PIN_5
#define U11_GPIO_Port GPIOB
#define U12_Pin GPIO_PIN_8
#define U12_GPIO_Port GPIOB
#define U13_Pin GPIO_PIN_9
#define U13_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
