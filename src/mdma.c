/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : mdma.c
  * Description        : This file provides code for the configuration
  *                      of all the requested global MDMA transfers.
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
#include "mdma.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure MDMA                                                              */
/*----------------------------------------------------------------------------*/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
MDMA_HandleTypeDef hmdma_mdma_channel0_sdmmc1_command_end_0;
MDMA_LinkNodeTypeDef node_mdma_channel0_sdmmc1_dma_endbuffer_1;
MDMA_LinkNodeTypeDef node_mdma_channel0_sdmmc1_end_data_2;
MDMA_HandleTypeDef hmdma_mdma_channel1_sw_0;
MDMA_LinkNodeTypeDef node_mdma_channel1_dma2_stream5_tc_1;
MDMA_HandleTypeDef hmdma_mdma_channel2_dma1_stream0_tc_0;

/**
  * Enable MDMA controller clock
  * Configure MDMA for global transfers
  *   hmdma_mdma_channel0_sdmmc1_command_end_0
  *   node_mdma_channel0_sdmmc1_dma_endbuffer_1
  *   node_mdma_channel0_sdmmc1_end_data_2
  *   hmdma_mdma_channel1_sw_0
  *   node_mdma_channel1_dma2_stream5_tc_1
  *   hmdma_mdma_channel2_dma1_stream0_tc_0
  */
void MX_MDMA_Init(void)
{

  /* MDMA controller clock enable */
  __HAL_RCC_MDMA_CLK_ENABLE();
  /* Local variables */
  MDMA_LinkNodeConfTypeDef nodeConfig;

  /* Configure MDMA channel MDMA_Channel0 */
  /* Configure MDMA request hmdma_mdma_channel0_sdmmc1_command_end_0 on MDMA_Channel0 */
  hmdma_mdma_channel0_sdmmc1_command_end_0.Instance = MDMA_Channel0;
  hmdma_mdma_channel0_sdmmc1_command_end_0.Init.Request = MDMA_REQUEST_SDMMC1_COMMAND_END;
  hmdma_mdma_channel0_sdmmc1_command_end_0.Init.TransferTriggerMode = MDMA_BLOCK_TRANSFER;
  hmdma_mdma_channel0_sdmmc1_command_end_0.Init.Priority = MDMA_PRIORITY_LOW;
  hmdma_mdma_channel0_sdmmc1_command_end_0.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
  hmdma_mdma_channel0_sdmmc1_command_end_0.Init.SourceInc = MDMA_SRC_INC_BYTE;
  hmdma_mdma_channel0_sdmmc1_command_end_0.Init.DestinationInc = MDMA_DEST_INC_BYTE;
  hmdma_mdma_channel0_sdmmc1_command_end_0.Init.SourceDataSize = MDMA_SRC_DATASIZE_BYTE;
  hmdma_mdma_channel0_sdmmc1_command_end_0.Init.DestDataSize = MDMA_DEST_DATASIZE_BYTE;
  hmdma_mdma_channel0_sdmmc1_command_end_0.Init.DataAlignment = MDMA_DATAALIGN_PACKENABLE;
  hmdma_mdma_channel0_sdmmc1_command_end_0.Init.BufferTransferLength = 128;
  hmdma_mdma_channel0_sdmmc1_command_end_0.Init.SourceBurst = MDMA_SOURCE_BURST_SINGLE;
  hmdma_mdma_channel0_sdmmc1_command_end_0.Init.DestBurst = MDMA_DEST_BURST_SINGLE;
  hmdma_mdma_channel0_sdmmc1_command_end_0.Init.SourceBlockAddressOffset = 0;
  hmdma_mdma_channel0_sdmmc1_command_end_0.Init.DestBlockAddressOffset = 0;
  if (HAL_MDMA_Init(&hmdma_mdma_channel0_sdmmc1_command_end_0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure post request address and data masks */
  if (HAL_MDMA_ConfigPostRequestMask(&hmdma_mdma_channel0_sdmmc1_command_end_0, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Initialize MDMA link node according to specified parameters */
  nodeConfig.Init.Request = MDMA_REQUEST_SDMMC1_DMA_ENDBUFFER;
  nodeConfig.Init.TransferTriggerMode = MDMA_BLOCK_TRANSFER;
  nodeConfig.Init.Priority = MDMA_PRIORITY_LOW;
  nodeConfig.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
  nodeConfig.Init.SourceInc = MDMA_SRC_INC_BYTE;
  nodeConfig.Init.DestinationInc = MDMA_DEST_INC_BYTE;
  nodeConfig.Init.SourceDataSize = MDMA_SRC_DATASIZE_BYTE;
  nodeConfig.Init.DestDataSize = MDMA_DEST_DATASIZE_BYTE;
  nodeConfig.Init.DataAlignment = MDMA_DATAALIGN_PACKENABLE;
  nodeConfig.Init.BufferTransferLength = 128;
  nodeConfig.Init.SourceBurst = MDMA_SOURCE_BURST_SINGLE;
  nodeConfig.Init.DestBurst = MDMA_DEST_BURST_SINGLE;
  nodeConfig.Init.SourceBlockAddressOffset = 0;
  nodeConfig.Init.DestBlockAddressOffset = 0;
  nodeConfig.PostRequestMaskAddress = 0;
  nodeConfig.PostRequestMaskData = 0;
  nodeConfig.SrcAddress = 0;
  nodeConfig.DstAddress = 0;
  nodeConfig.BlockDataLength = 0;
  nodeConfig.BlockCount = 0;
  if (HAL_MDMA_LinkedList_CreateNode(&node_mdma_channel0_sdmmc1_dma_endbuffer_1, &nodeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN mdma_channel0_sdmmc1_dma_endbuffer_1 */

  /* USER CODE END mdma_channel0_sdmmc1_dma_endbuffer_1 */

  /* Connect a node to the linked list */
  if (HAL_MDMA_LinkedList_AddNode(&hmdma_mdma_channel0_sdmmc1_command_end_0, &node_mdma_channel0_sdmmc1_dma_endbuffer_1, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Initialize MDMA link node according to specified parameters */
  nodeConfig.Init.Request = MDMA_REQUEST_SDMMC1_END_DATA;
  nodeConfig.Init.TransferTriggerMode = MDMA_BLOCK_TRANSFER;
  nodeConfig.Init.Priority = MDMA_PRIORITY_LOW;
  nodeConfig.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
  nodeConfig.Init.SourceInc = MDMA_SRC_INC_BYTE;
  nodeConfig.Init.DestinationInc = MDMA_DEST_INC_BYTE;
  nodeConfig.Init.SourceDataSize = MDMA_SRC_DATASIZE_BYTE;
  nodeConfig.Init.DestDataSize = MDMA_DEST_DATASIZE_BYTE;
  nodeConfig.Init.DataAlignment = MDMA_DATAALIGN_PACKENABLE;
  nodeConfig.Init.BufferTransferLength = 128;
  nodeConfig.Init.SourceBurst = MDMA_SOURCE_BURST_SINGLE;
  nodeConfig.Init.DestBurst = MDMA_DEST_BURST_SINGLE;
  nodeConfig.Init.SourceBlockAddressOffset = 0;
  nodeConfig.Init.DestBlockAddressOffset = 0;
  nodeConfig.PostRequestMaskAddress = 0;
  nodeConfig.PostRequestMaskData = 0;
  nodeConfig.SrcAddress = 0;
  nodeConfig.DstAddress = 0;
  nodeConfig.BlockDataLength = 0;
  nodeConfig.BlockCount = 0;
  if (HAL_MDMA_LinkedList_CreateNode(&node_mdma_channel0_sdmmc1_end_data_2, &nodeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN mdma_channel0_sdmmc1_end_data_2 */

  /* USER CODE END mdma_channel0_sdmmc1_end_data_2 */

  /* Connect a node to the linked list */
  if (HAL_MDMA_LinkedList_AddNode(&hmdma_mdma_channel0_sdmmc1_command_end_0, &node_mdma_channel0_sdmmc1_end_data_2, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure MDMA channel MDMA_Channel1 */
  /* Configure MDMA request hmdma_mdma_channel1_sw_0 on MDMA_Channel1 */
  hmdma_mdma_channel1_sw_0.Instance = MDMA_Channel1;
  hmdma_mdma_channel1_sw_0.Init.Request = MDMA_REQUEST_SW;
  hmdma_mdma_channel1_sw_0.Init.TransferTriggerMode = MDMA_BLOCK_TRANSFER;
  hmdma_mdma_channel1_sw_0.Init.Priority = MDMA_PRIORITY_LOW;
  hmdma_mdma_channel1_sw_0.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
  hmdma_mdma_channel1_sw_0.Init.SourceInc = MDMA_SRC_INC_BYTE;
  hmdma_mdma_channel1_sw_0.Init.DestinationInc = MDMA_DEST_INC_BYTE;
  hmdma_mdma_channel1_sw_0.Init.SourceDataSize = MDMA_SRC_DATASIZE_BYTE;
  hmdma_mdma_channel1_sw_0.Init.DestDataSize = MDMA_DEST_DATASIZE_BYTE;
  hmdma_mdma_channel1_sw_0.Init.DataAlignment = MDMA_DATAALIGN_PACKENABLE;
  hmdma_mdma_channel1_sw_0.Init.BufferTransferLength = 128;
  hmdma_mdma_channel1_sw_0.Init.SourceBurst = MDMA_SOURCE_BURST_SINGLE;
  hmdma_mdma_channel1_sw_0.Init.DestBurst = MDMA_DEST_BURST_SINGLE;
  hmdma_mdma_channel1_sw_0.Init.SourceBlockAddressOffset = 0;
  hmdma_mdma_channel1_sw_0.Init.DestBlockAddressOffset = 0;
  if (HAL_MDMA_Init(&hmdma_mdma_channel1_sw_0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Initialize MDMA link node according to specified parameters */
  nodeConfig.Init.Request = MDMA_REQUEST_DMA2_Stream5_TC;
  nodeConfig.Init.TransferTriggerMode = MDMA_BLOCK_TRANSFER;
  nodeConfig.Init.Priority = MDMA_PRIORITY_LOW;
  nodeConfig.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
  nodeConfig.Init.SourceInc = MDMA_SRC_INC_BYTE;
  nodeConfig.Init.DestinationInc = MDMA_DEST_INC_BYTE;
  nodeConfig.Init.SourceDataSize = MDMA_SRC_DATASIZE_BYTE;
  nodeConfig.Init.DestDataSize = MDMA_DEST_DATASIZE_BYTE;
  nodeConfig.Init.DataAlignment = MDMA_DATAALIGN_PACKENABLE;
  nodeConfig.Init.BufferTransferLength = 128;
  nodeConfig.Init.SourceBurst = MDMA_SOURCE_BURST_SINGLE;
  nodeConfig.Init.DestBurst = MDMA_DEST_BURST_SINGLE;
  nodeConfig.Init.SourceBlockAddressOffset = 0;
  nodeConfig.Init.DestBlockAddressOffset = 0;
  nodeConfig.PostRequestMaskAddress = 0;
  nodeConfig.PostRequestMaskData = 0;
  nodeConfig.SrcAddress = 0;
  nodeConfig.DstAddress = 0;
  nodeConfig.BlockDataLength = 0;
  nodeConfig.BlockCount = 0;
  if (HAL_MDMA_LinkedList_CreateNode(&node_mdma_channel1_dma2_stream5_tc_1, &nodeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN mdma_channel1_dma2_stream5_tc_1 */

  /* USER CODE END mdma_channel1_dma2_stream5_tc_1 */

  /* Connect a node to the linked list */
  if (HAL_MDMA_LinkedList_AddNode(&hmdma_mdma_channel1_sw_0, &node_mdma_channel1_dma2_stream5_tc_1, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure MDMA channel MDMA_Channel2 */
  /* Configure MDMA request hmdma_mdma_channel2_dma1_stream0_tc_0 on MDMA_Channel2 */
  hmdma_mdma_channel2_dma1_stream0_tc_0.Instance = MDMA_Channel2;
  hmdma_mdma_channel2_dma1_stream0_tc_0.Init.Request = MDMA_REQUEST_DMA1_Stream0_TC;
  hmdma_mdma_channel2_dma1_stream0_tc_0.Init.TransferTriggerMode = MDMA_BLOCK_TRANSFER;
  hmdma_mdma_channel2_dma1_stream0_tc_0.Init.Priority = MDMA_PRIORITY_LOW;
  hmdma_mdma_channel2_dma1_stream0_tc_0.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
  hmdma_mdma_channel2_dma1_stream0_tc_0.Init.SourceInc = MDMA_SRC_INC_BYTE;
  hmdma_mdma_channel2_dma1_stream0_tc_0.Init.DestinationInc = MDMA_DEST_INC_BYTE;
  hmdma_mdma_channel2_dma1_stream0_tc_0.Init.SourceDataSize = MDMA_SRC_DATASIZE_BYTE;
  hmdma_mdma_channel2_dma1_stream0_tc_0.Init.DestDataSize = MDMA_DEST_DATASIZE_BYTE;
  hmdma_mdma_channel2_dma1_stream0_tc_0.Init.DataAlignment = MDMA_DATAALIGN_PACKENABLE;
  hmdma_mdma_channel2_dma1_stream0_tc_0.Init.BufferTransferLength = 128;
  hmdma_mdma_channel2_dma1_stream0_tc_0.Init.SourceBurst = MDMA_SOURCE_BURST_SINGLE;
  hmdma_mdma_channel2_dma1_stream0_tc_0.Init.DestBurst = MDMA_DEST_BURST_SINGLE;
  hmdma_mdma_channel2_dma1_stream0_tc_0.Init.SourceBlockAddressOffset = 0;
  hmdma_mdma_channel2_dma1_stream0_tc_0.Init.DestBlockAddressOffset = 0;
  if (HAL_MDMA_Init(&hmdma_mdma_channel2_dma1_stream0_tc_0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure post request address and data masks */
  if (HAL_MDMA_ConfigPostRequestMask(&hmdma_mdma_channel2_dma1_stream0_tc_0, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* MDMA interrupt initialization */
  /* MDMA_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MDMA_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(MDMA_IRQn);

}
/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

/**
  * @}
  */

/**
  * @}
  */

