/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f3xx_hal.h"

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
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOF
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_3
#define VCP_RX_GPIO_Port GPIOA
#define SILNIK_P_2_Pin GPIO_PIN_4
#define SILNIK_P_2_GPIO_Port GPIOA
#define TOF_XSHUT_2_Pin GPIO_PIN_7
#define TOF_XSHUT_2_GPIO_Port GPIOA
#define TOF_INT_0_Pin GPIO_PIN_0
#define TOF_INT_0_GPIO_Port GPIOB
#define TOF_INT_0_EXTI_IRQn EXTI0_IRQn
#define TOF_INT_2_Pin GPIO_PIN_1
#define TOF_INT_2_GPIO_Port GPIOB
#define TOF_INT_2_EXTI_IRQn EXTI1_IRQn
#define SILNIK_L_2_Pin GPIO_PIN_8
#define SILNIK_L_2_GPIO_Port GPIOA
#define SILNIK_L_1_Pin GPIO_PIN_9
#define SILNIK_L_1_GPIO_Port GPIOA
#define TOF_XSHUT_1_Pin GPIO_PIN_10
#define TOF_XSHUT_1_GPIO_Port GPIOA
#define TOF_INT_1_Pin GPIO_PIN_11
#define TOF_INT_1_GPIO_Port GPIOA
#define TOF_INT_1_EXTI_IRQn EXTI15_10_IRQn
#define TOF_XSHUT_0_Pin GPIO_PIN_12
#define TOF_XSHUT_0_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define SILNIK_P_1_Pin GPIO_PIN_3
#define SILNIK_P_1_GPIO_Port GPIOB
#define PWM_L_Pin GPIO_PIN_4
#define PWM_L_GPIO_Port GPIOB
#define PWM_P_Pin GPIO_PIN_5
#define PWM_P_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
