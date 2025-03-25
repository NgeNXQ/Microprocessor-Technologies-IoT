/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define GPIO_Output_Pin GPIO_PIN_1
#define GPIO_Output_GPIO_Port GPIOA
#define GPIO_OutputA2_Pin GPIO_PIN_2
#define GPIO_OutputA2_GPIO_Port GPIOA
#define GPIO_OutputA3_Pin GPIO_PIN_3
#define GPIO_OutputA3_GPIO_Port GPIOA
#define GPIO_OutputA4_Pin GPIO_PIN_4
#define GPIO_OutputA4_GPIO_Port GPIOA
#define GPIO_OutputA5_Pin GPIO_PIN_5
#define GPIO_OutputA5_GPIO_Port GPIOA
#define TIM1_CH1N_Pin GPIO_PIN_7
#define TIM1_CH1N_GPIO_Port GPIOA
#define GPIO_OutputB1_Pin GPIO_PIN_1
#define GPIO_OutputB1_GPIO_Port GPIOB
#define GPIO_OutputB2_Pin GPIO_PIN_2
#define GPIO_OutputB2_GPIO_Port GPIOB
#define TIM1_CH1_Pin GPIO_PIN_8
#define TIM1_CH1_GPIO_Port GPIOA
#define GPIO_EXTI12_Pin GPIO_PIN_12
#define GPIO_EXTI12_GPIO_Port GPIOA
#define GPIO_EXTI12_EXTI_IRQn EXTI15_10_IRQn
#define GPIO_EXTI13_Pin GPIO_PIN_13
#define GPIO_EXTI13_GPIO_Port GPIOA
#define GPIO_EXTI13_EXTI_IRQn EXTI15_10_IRQn
#define GPIO_OutputB3_Pin GPIO_PIN_3
#define GPIO_OutputB3_GPIO_Port GPIOB
#define GPIO_OutputB4_Pin GPIO_PIN_4
#define GPIO_OutputB4_GPIO_Port GPIOB
#define GPIO_OutputB5_Pin GPIO_PIN_5
#define GPIO_OutputB5_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
