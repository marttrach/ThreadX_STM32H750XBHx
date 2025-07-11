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
#include "stm32h7xx_hal.h"

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
#define PWM_3_Pin GPIO_PIN_15
#define PWM_3_GPIO_Port GPIOA
#define SCL_1_Pin GPIO_PIN_6
#define SCL_1_GPIO_Port GPIOB
#define SDA_1_Pin GPIO_PIN_7
#define SDA_1_GPIO_Port GPIOB
#define PWM_4_Pin GPIO_PIN_3
#define PWM_4_GPIO_Port GPIOB
#define UD3_Pin GPIO_PIN_3
#define UD3_GPIO_Port GPIOE
#define FDCAN_TX_Pin GPIO_PIN_9
#define FDCAN_TX_GPIO_Port GPIOB
#define FDCAN_RX_Pin GPIO_PIN_8
#define FDCAN_RX_GPIO_Port GPIOB
#define SDIO1_CD_Pin GPIO_PIN_14
#define SDIO1_CD_GPIO_Port GPIOG
#define UE3_Pin GPIO_PIN_13
#define UE3_GPIO_Port GPIOC
#define UART7_RX_Pin GPIO_PIN_8
#define UART7_RX_GPIO_Port GPIOA
#define PWM_6_Pin GPIO_PIN_7
#define PWM_6_GPIO_Port GPIOC
#define UART7_TX_Pin GPIO_PIN_7
#define UART7_TX_GPIO_Port GPIOF
#define ADC_4_Pin GPIO_PIN_0
#define ADC_4_GPIO_Port GPIOC
#define ADC_5_Pin GPIO_PIN_1
#define ADC_5_GPIO_Port GPIOC
#define ADC_0_Pin GPIO_PIN_2
#define ADC_0_GPIO_Port GPIOC
#define UART8_TX_Pin GPIO_PIN_9
#define UART8_TX_GPIO_Port GPIOJ
#define PWM_1_Pin GPIO_PIN_2
#define PWM_1_GPIO_Port GPIOA
#define UART8_RX_Pin GPIO_PIN_8
#define UART8_RX_GPIO_Port GPIOJ
#define SCL_2_Pin GPIO_PIN_4
#define SCL_2_GPIO_Port GPIOH
#define SDA_2_Pin GPIO_PIN_5
#define SDA_2_GPIO_Port GPIOH
#define ADC_1_Pin GPIO_PIN_6
#define ADC_1_GPIO_Port GPIOA
#define ADC_2_Pin GPIO_PIN_4
#define ADC_2_GPIO_Port GPIOC
#define ADC_3_Pin GPIO_PIN_1
#define ADC_3_GPIO_Port GPIOB
#define PWM_5_Pin GPIO_PIN_6
#define PWM_5_GPIO_Port GPIOH
#define UART5_RX_Pin GPIO_PIN_12
#define UART5_RX_GPIO_Port GPIOB
#define DAC_1_GPIO5_Pin GPIO_PIN_4
#define DAC_1_GPIO5_GPIO_Port GPIOA
#define PWM_2_Pin GPIO_PIN_0
#define PWM_2_GPIO_Port GPIOB
#define UART5_TX_Pin GPIO_PIN_13
#define UART5_TX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
