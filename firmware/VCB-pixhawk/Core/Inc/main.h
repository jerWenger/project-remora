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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
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
#define SENSE_N_Pin GPIO_PIN_2
#define SENSE_N_GPIO_Port GPIOE
#define CONT_TX_Pin GPIO_PIN_2
#define CONT_TX_GPIO_Port GPIOA
#define CONT_RX_Pin GPIO_PIN_3
#define CONT_RX_GPIO_Port GPIOA
#define SENSE_P_Pin GPIO_PIN_6
#define SENSE_P_GPIO_Port GPIOA
#define SENSE_NA7_Pin GPIO_PIN_7
#define SENSE_NA7_GPIO_Port GPIOA
#define LED_B_Pin GPIO_PIN_0
#define LED_B_GPIO_Port GPIOB
#define RS485_1_RX_Pin GPIO_PIN_7
#define RS485_1_RX_GPIO_Port GPIOE
#define RS485_1_TX_Pin GPIO_PIN_8
#define RS485_1_TX_GPIO_Port GPIOE
#define RS485_1_DE_Pin GPIO_PIN_9
#define RS485_1_DE_GPIO_Port GPIOE
#define RS485_2_TX_Pin GPIO_PIN_10
#define RS485_2_TX_GPIO_Port GPIOB
#define RS485_2_RX_Pin GPIO_PIN_11
#define RS485_2_RX_GPIO_Port GPIOB
#define RS485_3_RX_Pin GPIO_PIN_12
#define RS485_3_RX_GPIO_Port GPIOB
#define RS485_3_TX_Pin GPIO_PIN_13
#define RS485_3_TX_GPIO_Port GPIOB
#define RS485_4_TX_Pin GPIO_PIN_14
#define RS485_4_TX_GPIO_Port GPIOB
#define RS485_4_RX_Pin GPIO_PIN_15
#define RS485_4_RX_GPIO_Port GPIOB
#define RS485_2_DE_Pin GPIO_PIN_12
#define RS485_2_DE_GPIO_Port GPIOD
#define RS485_5_DE_Pin GPIO_PIN_13
#define RS485_5_DE_GPIO_Port GPIOD
#define RS485_5_RX_Pin GPIO_PIN_14
#define RS485_5_RX_GPIO_Port GPIOD
#define RS485_5_TX_Pin GPIO_PIN_15
#define RS485_5_TX_GPIO_Port GPIOD
#define LED_R_Pin GPIO_PIN_6
#define LED_R_GPIO_Port GPIOC
#define LED_G_Pin GPIO_PIN_7
#define LED_G_GPIO_Port GPIOC
#define RS485_3_DE_Pin GPIO_PIN_8
#define RS485_3_DE_GPIO_Port GPIOC
#define SDN_ESTOP_OUT_SENSE_Pin GPIO_PIN_9
#define SDN_ESTOP_OUT_SENSE_GPIO_Port GPIOA
#define SDN_TSMS_OUT_SENSE_Pin GPIO_PIN_10
#define SDN_TSMS_OUT_SENSE_GPIO_Port GPIOA
#define RS485_4_DE_Pin GPIO_PIN_12
#define RS485_4_DE_GPIO_Port GPIOA
#define CTL4_Pin GPIO_PIN_10
#define CTL4_GPIO_Port GPIOC
#define CTL3_Pin GPIO_PIN_11
#define CTL3_GPIO_Port GPIOC
#define CTL2_Pin GPIO_PIN_12
#define CTL2_GPIO_Port GPIOC
#define CTL1_Pin GPIO_PIN_0
#define CTL1_GPIO_Port GPIOD
#define HSI_3_3_Pin GPIO_PIN_3
#define HSI_3_3_GPIO_Port GPIOD
#define LSI_3_3_Pin GPIO_PIN_4
#define LSI_3_3_GPIO_Port GPIOD
#define LSC_CONTROL_Pin GPIO_PIN_5
#define LSC_CONTROL_GPIO_Port GPIOD
#define HSC_CONTROL_Pin GPIO_PIN_6
#define HSC_CONTROL_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
