/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : mdma.h
  * Description        : This file contains all the function prototypes for
  *                      the mdma.c file
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
#ifndef __mdma_H
#define __mdma_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* MDMA transfer handles -----------------------------------------------------*/
extern MDMA_HandleTypeDef hmdma_mdma_channel0_sdmmc1_end_data_0;
extern MDMA_HandleTypeDef hmdma_mdma_channel1_dma1_stream4_tc_0;
extern MDMA_HandleTypeDef hmdma_mdma_channel2_dma1_stream5_tc_0;
extern MDMA_HandleTypeDef hmdma_mdma_channel4_dma1_stream1_tc_0;
extern MDMA_HandleTypeDef hmdma_mdma_channel5_dma1_stream2_tc_0;
extern MDMA_HandleTypeDef hmdma_mdma_channel6_dma1_stream3_tc_0;
extern MDMA_HandleTypeDef hmdma_mdma_channel7_dma2_stream1_tc_0;
extern MDMA_HandleTypeDef hmdma_mdma_channel8_dma2_stream4_tc_0;
extern MDMA_HandleTypeDef hmdma_mdma_channel10_dma1_stream7_tc_0;
extern MDMA_HandleTypeDef hmdma_mdma_channel11_dma2_stream0_tc_0;
extern MDMA_HandleTypeDef hmdma_mdma_channel12_dma2_stream2_tc_0;
extern MDMA_HandleTypeDef hmdma_mdma_channel13_dma2_stream3_tc_0;
extern MDMA_HandleTypeDef hmdma_mdma_channel14_dma2_stream7_tc_0;
extern MDMA_HandleTypeDef hmdma_mdma_channel15_dma2_stream5_tc_0;

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_MDMA_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __mdma_H */

/**
  * @}
  */
