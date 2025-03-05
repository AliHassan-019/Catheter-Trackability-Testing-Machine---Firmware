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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Water_Out_Pin GPIO_PIN_14
#define Water_Out_GPIO_Port GPIOC
#define Water_In_Pin GPIO_PIN_0
#define Water_In_GPIO_Port GPIOF
#define Min_Lim_Pin GPIO_PIN_2
#define Min_Lim_GPIO_Port GPIOF
#define Min_Lim_EXTI_IRQn EXTI2_IRQn
#define Max_Lim_Pin GPIO_PIN_4
#define Max_Lim_GPIO_Port GPIOF
#define Max_Lim_EXTI_IRQn EXTI4_IRQn
#define Heater_Switch_Pin GPIO_PIN_7
#define Heater_Switch_GPIO_Port GPIOF
#define Loadcell_Pin GPIO_PIN_2
#define Loadcell_GPIO_Port GPIOA
#define Temp_Read_Pin GPIO_PIN_6
#define Temp_Read_GPIO_Port GPIOA
#define Valve1_Pin GPIO_PIN_12
#define Valve1_GPIO_Port GPIOE
#define Valve2_Pin GPIO_PIN_14
#define Valve2_GPIO_Port GPIOE
#define Heater_Pin GPIO_PIN_5
#define Heater_GPIO_Port GPIOG
#define Pump_Out_Pin GPIO_PIN_7
#define Pump_Out_GPIO_Port GPIOG
#define Pulse_Pin GPIO_PIN_8
#define Pulse_GPIO_Port GPIOA
#define En_Pin GPIO_PIN_10
#define En_GPIO_Port GPIOA
#define Dir_Pin GPIO_PIN_12
#define Dir_GPIO_Port GPIOA
#define NSS_Pin GPIO_PIN_15
#define NSS_GPIO_Port GPIOA
#define SCK_Pin GPIO_PIN_10
#define SCK_GPIO_Port GPIOC
#define MISO_Pin GPIO_PIN_11
#define MISO_GPIO_Port GPIOC
#define MOSI_Pin GPIO_PIN_12
#define MOSI_GPIO_Port GPIOC
#define Pump_In_Pin GPIO_PIN_9
#define Pump_In_GPIO_Port GPIOG
#define Air_Valve_Pin GPIO_PIN_11
#define Air_Valve_GPIO_Port GPIOG
#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOG
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOG

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
