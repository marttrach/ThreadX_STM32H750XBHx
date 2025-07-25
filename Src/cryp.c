/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    cryp.c
  * @brief   This file provides code for the configuration
  *          of the CRYP instances.
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
/* Includes ------------------------------------------------------------------*/
#include "cryp.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

CRYP_HandleTypeDef hcryp;
__ALIGN_BEGIN static const uint32_t pKeyCRYP[4] __ALIGN_END = {
                            0x00000000,0x00000000,0x00000000,0x00000000};
__ALIGN_BEGIN static const uint32_t pInitVectCRYP[4] __ALIGN_END = {
                            0x00000000,0x00000000,0x00000000,0x00000002};
__ALIGN_BEGIN static const uint32_t HeaderCRYP[1] __ALIGN_END = {
                            0x00000000};
DMA_HandleTypeDef hdma_cryp_out;
DMA_HandleTypeDef hdma_cryp_in;

/* CRYP init function */
void MX_CRYP_Init(void)
{

  /* USER CODE BEGIN CRYP_Init 0 */

  /* USER CODE END CRYP_Init 0 */

  /* USER CODE BEGIN CRYP_Init 1 */

  /* USER CODE END CRYP_Init 1 */
  hcryp.Instance = CRYP;
  hcryp.Init.DataType = CRYP_DATATYPE_32B;
  hcryp.Init.KeySize = CRYP_KEYSIZE_128B;
  hcryp.Init.pKey = (uint32_t *)pKeyCRYP;
  hcryp.Init.pInitVect = (uint32_t *)pInitVectCRYP;
  hcryp.Init.Algorithm = CRYP_AES_GCM;
  hcryp.Init.Header = (uint32_t *)HeaderCRYP;
  hcryp.Init.HeaderSize = 1;
  if (HAL_CRYP_Init(&hcryp) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRYP_Init 2 */

  /* USER CODE END CRYP_Init 2 */

}

void HAL_CRYP_MspInit(CRYP_HandleTypeDef* crypHandle)
{

  if(crypHandle->Instance==CRYP)
  {
  /* USER CODE BEGIN CRYP_MspInit 0 */

  /* USER CODE END CRYP_MspInit 0 */
    /* CRYP clock enable */
    __HAL_RCC_CRYP_CLK_ENABLE();

    /* CRYP DMA Init */
    /* CRYP_OUT Init */
    hdma_cryp_out.Instance = DMA2_Stream7;
    hdma_cryp_out.Init.Request = DMA_REQUEST_CRYP_OUT;
    hdma_cryp_out.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_cryp_out.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_cryp_out.Init.MemInc = DMA_MINC_ENABLE;
    hdma_cryp_out.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_cryp_out.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_cryp_out.Init.Mode = DMA_NORMAL;
    hdma_cryp_out.Init.Priority = DMA_PRIORITY_LOW;
    hdma_cryp_out.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_cryp_out) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(crypHandle,hdmaout,hdma_cryp_out);

    /* CRYP_IN Init */
    hdma_cryp_in.Instance = DMA2_Stream3;
    hdma_cryp_in.Init.Request = DMA_REQUEST_CRYP_IN;
    hdma_cryp_in.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_cryp_in.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_cryp_in.Init.MemInc = DMA_MINC_ENABLE;
    hdma_cryp_in.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_cryp_in.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_cryp_in.Init.Mode = DMA_NORMAL;
    hdma_cryp_in.Init.Priority = DMA_PRIORITY_LOW;
    hdma_cryp_in.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_cryp_in) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(crypHandle,hdmain,hdma_cryp_in);

    /* CRYP interrupt Init */
    HAL_NVIC_SetPriority(CRYP_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CRYP_IRQn);
  /* USER CODE BEGIN CRYP_MspInit 1 */

  /* USER CODE END CRYP_MspInit 1 */
  }
}

void HAL_CRYP_MspDeInit(CRYP_HandleTypeDef* crypHandle)
{

  if(crypHandle->Instance==CRYP)
  {
  /* USER CODE BEGIN CRYP_MspDeInit 0 */

  /* USER CODE END CRYP_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CRYP_CLK_DISABLE();

    /* CRYP DMA DeInit */
    HAL_DMA_DeInit(crypHandle->hdmaout);
    HAL_DMA_DeInit(crypHandle->hdmain);

    /* CRYP interrupt Deinit */
    HAL_NVIC_DisableIRQ(CRYP_IRQn);
  /* USER CODE BEGIN CRYP_MspDeInit 1 */

  /* USER CODE END CRYP_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
