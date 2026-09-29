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
#define DJ_EN_Pin GPIO_PIN_4
#define DJ_EN_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_12
#define LED_GPIO_Port GPIOB
#define HANDBREAK_Pin GPIO_PIN_14
#define HANDBREAK_GPIO_Port GPIOB
#define FYLimit_Down_Pin GPIO_PIN_15
#define FYLimit_Down_GPIO_Port GPIOB
#define FYLimit_Up_Pin GPIO_PIN_8
#define FYLimit_Up_GPIO_Port GPIOC
#define FYLimit_Lock_Pin GPIO_PIN_9
#define FYLimit_Lock_GPIO_Port GPIOC
#define FY_HALock_Pin GPIO_PIN_8
#define FY_HALock_GPIO_Port GPIOA
#define FW_HALock_Pin GPIO_PIN_9
#define FW_HALock_GPIO_Port GPIOA
#define INT2_Pin GPIO_PIN_12
#define INT2_GPIO_Port GPIOA
#define INT1_Pin GPIO_PIN_15
#define INT1_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
