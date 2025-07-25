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
MDMA_HandleTypeDef hmdma_mdma_channel0_sdmmc1_end_data_0;
MDMA_HandleTypeDef hmdma_mdma_channel1_dma1_stream4_tc_0;
MDMA_HandleTypeDef hmdma_mdma_channel2_dma1_stream5_tc_0;
MDMA_HandleTypeDef hmdma_mdma_channel3_dma1_stream0_tc_0;
MDMA_HandleTypeDef hmdma_mdma_channel4_dma1_stream1_tc_0;
MDMA_HandleTypeDef hmdma_mdma_channel5_dma1_stream2_tc_0;
MDMA_HandleTypeDef hmdma_mdma_channel6_dma1_stream3_tc_0;
MDMA_HandleTypeDef hmdma_mdma_channel7_dma2_stream1_tc_0;
MDMA_HandleTypeDef hmdma_mdma_channel8_dma2_stream4_tc_0;
MDMA_HandleTypeDef hmdma_mdma_channel9_dma1_stream6_tc_0;
MDMA_HandleTypeDef hmdma_mdma_channel10_dma1_stream7_tc_0;
MDMA_HandleTypeDef hmdma_mdma_channel11_dma2_stream0_tc_0;
MDMA_HandleTypeDef hmdma_mdma_channel12_dma2_stream2_tc_0;
MDMA_HandleTypeDef hmdma_mdma_channel13_dma2_stream3_tc_0;
MDMA_HandleTypeDef hmdma_mdma_channel14_dma2_stream7_tc_0;
MDMA_HandleTypeDef hmdma_mdma_channel15_dma2_stream5_tc_0;

/**
  * Enable MDMA controller clock
  * Configure MDMA for global transfers
  *   hmdma_mdma_channel0_sdmmc1_end_data_0
  *   hmdma_mdma_channel1_dma1_stream4_tc_0
  *   hmdma_mdma_channel2_dma1_stream5_tc_0
  *   hmdma_mdma_channel3_dma1_stream0_tc_0
  *   hmdma_mdma_channel4_dma1_stream1_tc_0
  *   hmdma_mdma_channel5_dma1_stream2_tc_0
  *   hmdma_mdma_channel6_dma1_stream3_tc_0
  *   hmdma_mdma_channel7_dma2_stream1_tc_0
  *   hmdma_mdma_channel8_dma2_stream4_tc_0
  *   hmdma_mdma_channel9_dma1_stream6_tc_0
  *   hmdma_mdma_channel10_dma1_stream7_tc_0
  *   hmdma_mdma_channel11_dma2_stream0_tc_0
  *   hmdma_mdma_channel12_dma2_stream2_tc_0
  *   hmdma_mdma_channel13_dma2_stream3_tc_0
  *   hmdma_mdma_channel14_dma2_stream7_tc_0
  *   hmdma_mdma_channel15_dma2_stream5_tc_0
  */
void MX_MDMA_Init(void)
{

  /* MDMA controller clock enable */
  __HAL_RCC_MDMA_CLK_ENABLE();
  /* Local variables */

  /* Configure MDMA channel MDMA_Channel0 */
  /* Configure MDMA request hmdma_mdma_channel0_sdmmc1_end_data_0 on MDMA_Channel0 */
  hmdma_mdma_channel0_sdmmc1_end_data_0.Instance = MDMA_Channel0;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.Request = MDMA_REQUEST_SDMMC1_END_DATA;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.TransferTriggerMode = MDMA_BLOCK_TRANSFER;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.Priority = MDMA_PRIORITY_HIGH;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.SourceInc = MDMA_SRC_INC_BYTE;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.DestinationInc = MDMA_DEST_INC_BYTE;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.SourceDataSize = MDMA_SRC_DATASIZE_BYTE;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.DestDataSize = MDMA_DEST_DATASIZE_BYTE;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.DataAlignment = MDMA_DATAALIGN_PACKENABLE;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.BufferTransferLength = 128;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.SourceBurst = MDMA_SOURCE_BURST_SINGLE;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.DestBurst = MDMA_DEST_BURST_SINGLE;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.SourceBlockAddressOffset = 0;
  hmdma_mdma_channel0_sdmmc1_end_data_0.Init.DestBlockAddressOffset = 0;
  if (HAL_MDMA_Init(&hmdma_mdma_channel0_sdmmc1_end_data_0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure post request address and data masks */
  if (HAL_MDMA_ConfigPostRequestMask(&hmdma_mdma_channel0_sdmmc1_end_data_0, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure MDMA channel MDMA_Channel1 */
  /* Configure MDMA request hmdma_mdma_channel1_dma1_stream4_tc_0 on MDMA_Channel1 */
  hmdma_mdma_channel1_dma1_stream4_tc_0.Instance = MDMA_Channel1;
  hmdma_mdma_channel1_dma1_stream4_tc_0.Init.Request = MDMA_REQUEST_DMA1_Stream4_TC;
  hmdma_mdma_channel1_dma1_stream4_tc_0.Init.TransferTriggerMode = MDMA_BLOCK_TRANSFER;
  hmdma_mdma_channel1_dma1_stream4_tc_0.Init.Priority = MDMA_PRIORITY_VERY_HIGH;
  hmdma_mdma_channel1_dma1_stream4_tc_0.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
  hmdma_mdma_channel1_dma1_stream4_tc_0.Init.SourceInc = MDMA_SRC_INC_BYTE;
  hmdma_mdma_channel1_dma1_stream4_tc_0.Init.DestinationInc = MDMA_DEST_INC_BYTE;
  hmdma_mdma_channel1_dma1_stream4_tc_0.Init.SourceDataSize = MDMA_SRC_DATASIZE_BYTE;
  hmdma_mdma_channel1_dma1_stream4_tc_0.Init.DestDataSize = MDMA_DEST_DATASIZE_BYTE;
  hmdma_mdma_channel1_dma1_stream4_tc_0.Init.DataAlignment = MDMA_DATAALIGN_PACKENABLE;
  hmdma_mdma_channel1_dma1_stream4_tc_0.Init.BufferTransferLength = 128;
  hmdma_mdma_channel1_dma1_stream4_tc_0.Init.SourceBurst = MDMA_SOURCE_BURST_SINGLE;
  hmdma_mdma_channel1_dma1_stream4_tc_0.Init.DestBurst = MDMA_DEST_BURST_SINGLE;
  hmdma_mdma_channel1_dma1_stream4_tc_0.Init.SourceBlockAddressOffset = 0;
  hmdma_mdma_channel1_dma1_stream4_tc_0.Init.DestBlockAddressOffset = 0;
  if (HAL_MDMA_Init(&hmdma_mdma_channel1_dma1_stream4_tc_0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure post request address and data masks */
  if (HAL_MDMA_ConfigPostRequestMask(&hmdma_mdma_channel1_dma1_stream4_tc_0, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure MDMA channel MDMA_Channel2 */
  /* Configure MDMA request hmdma_mdma_channel2_dma1_stream5_tc_0 on MDMA_Channel2 */
  hmdma_mdma_channel2_dma1_stream5_tc_0.Instance = MDMA_Channel2;
  hmdma_mdma_channel2_dma1_stream5_tc_0.Init.Request = MDMA_REQUEST_DMA1_Stream5_TC;
  hmdma_mdma_channel2_dma1_stream5_tc_0.Init.TransferTriggerMode = MDMA_BLOCK_TRANSFER;
  hmdma_mdma_channel2_dma1_stream5_tc_0.Init.Priority = MDMA_PRIORITY_HIGH;
  hmdma_mdma_channel2_dma1_stream5_tc_0.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
  hmdma_mdma_channel2_dma1_stream5_tc_0.Init.SourceInc = MDMA_SRC_INC_BYTE;
  hmdma_mdma_channel2_dma1_stream5_tc_0.Init.DestinationInc = MDMA_DEST_INC_BYTE;
  hmdma_mdma_channel2_dma1_stream5_tc_0.Init.SourceDataSize = MDMA_SRC_DATASIZE_BYTE;
  hmdma_mdma_channel2_dma1_stream5_tc_0.Init.DestDataSize = MDMA_DEST_DATASIZE_BYTE;
  hmdma_mdma_channel2_dma1_stream5_tc_0.Init.DataAlignment = MDMA_DATAALIGN_PACKENABLE;
  hmdma_mdma_channel2_dma1_stream5_tc_0.Init.BufferTransferLength = 128;
  hmdma_mdma_channel2_dma1_stream5_tc_0.Init.SourceBurst = MDMA_SOURCE_BURST_SINGLE;
  hmdma_mdma_channel2_dma1_stream5_tc_0.Init.DestBurst = MDMA_DEST_BURST_SINGLE;
  hmdma_mdma_channel2_dma1_stream5_tc_0.Init.SourceBlockAddressOffset = 0;
  hmdma_mdma_channel2_dma1_stream5_tc_0.Init.DestBlockAddressOffset = 0;
  if (HAL_MDMA_Init(&hmdma_mdma_channel2_dma1_stream5_tc_0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure post request address and data masks */
  if (HAL_MDMA_ConfigPostRequestMask(&hmdma_mdma_channel2_dma1_stream5_tc_0, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure MDMA channel MDMA_Channel3 */
  /* Configure MDMA request hmdma_mdma_channel3_dma1_stream0_tc_0 on MDMA_Channel3 */
  hmdma_mdma_channel3_dma1_stream0_tc_0.Instance = MDMA_Channel3;
  hmdma_mdma_channel3_dma1_stream0_tc_0.Init.Request = MDMA_REQUEST_DMA1_Stream0_TC;
  hmdma_mdma_channel3_dma1_stream0_tc_0.Init.TransferTriggerMode = MDMA_BLOCK_TRANSFER;
  hmdma_mdma_channel3_dma1_stream0_tc_0.Init.Priority = MDMA_PRIORITY_LOW;
  hmdma_mdma_channel3_dma1_stream0_tc_0.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
  hmdma_mdma_channel3_dma1_stream0_tc_0.Init.SourceInc = MDMA_SRC_INC_BYTE;
  hmdma_mdma_channel3_dma1_stream0_tc_0.Init.DestinationInc = MDMA_DEST_INC_BYTE;
  hmdma_mdma_channel3_dma1_stream0_tc_0.Init.SourceDataSize = MDMA_SRC_DATASIZE_BYTE;
  hmdma_mdma_channel3_dma1_stream0_tc_0.Init.DestDataSize = MDMA_DEST_DATASIZE_BYTE;
  hmdma_mdma_channel3_dma1_stream0_tc_0.Init.DataAlignment = MDMA_DATAALIGN_PACKENABLE;
  hmdma_mdma_channel3_dma1_stream0_tc_0.Init.BufferTransferLength = 128;
  hmdma_mdma_channel3_dma1_stream0_tc_0.Init.SourceBurst = MDMA_SOURCE_BURST_SINGLE;
  hmdma_mdma_channel3_dma1_stream0_tc_0.Init.DestBurst = MDMA_DEST_BURST_SINGLE;
  hmdma_mdma_channel3_dma1_stream0_tc_0.Init.SourceBlockAddressOffset = 0;
  hmdma_mdma_channel3_dma1_stream0_tc_0.Init.DestBlockAddressOffset = 0;
  if (HAL_MDMA_Init(&hmdma_mdma_channel3_dma1_stream0_tc_0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure post request address and data masks */
  if (HAL_MDMA_ConfigPostRequestMask(&hmdma_mdma_channel3_dma1_stream0_tc_0, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure MDMA channel MDMA_Channel4 */
  /* Configure MDMA request hmdma_mdma_channel4_dma1_stream1_tc_0 on MDMA_Channel4 */
  hmdma_mdma_channel4_dma1_stream1_tc_0.Instance = MDMA_Channel4;
  hmdma_mdma_channel4_dma1_stream1_tc_0.Init.Request = MDMA_REQUEST_DMA1_Stream1_TC;
  hmdma_mdma_channel4_dma1_stream1_tc_0.Init.TransferTriggerMode = MDMA_BLOCK_TRANSFER;
  hmdma_mdma_channel4_dma1_stream1_tc_0.Init.Priority = MDMA_PRIORITY_LOW;
  hmdma_mdma_channel4_dma1_stream1_tc_0.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
  hmdma_mdma_channel4_dma1_stream1_tc_0.Init.SourceInc = MDMA_SRC_INC_BYTE;
  hmdma_mdma_channel4_dma1_stream1_tc_0.Init.DestinationInc = MDMA_DEST_INC_BYTE;
  hmdma_mdma_channel4_dma1_stream1_tc_0.Init.SourceDataSize = MDMA_SRC_DATASIZE_BYTE;
  hmdma_mdma_channel4_dma1_stream1_tc_0.Init.DestDataSize = MDMA_DEST_DATASIZE_BYTE;
  hmdma_mdma_channel4_dma1_stream1_tc_0.Init.DataAlignment = MDMA_DATAALIGN_PACKENABLE;
  hmdma_mdma_channel4_dma1_stream1_tc_0.Init.BufferTransferLength = 128;
  hmdma_mdma_channel4_dma1_stream1_tc_0.Init.SourceBurst = MDMA_SOURCE_BURST_SINGLE;
  hmdma_mdma_channel4_dma1_stream1_tc_0.Init.DestBurst = MDMA_DEST_BURST_SINGLE;
  hmdma_mdma_channel4_dma1_stream1_tc_0.Init.SourceBlockAddressOffset = 0;
  hmdma_mdma_channel4_dma1_stream1_tc_0.Init.DestBlockAddressOffset = 0;
  if (HAL_MDMA_Init(&hmdma_mdma_channel4_dma1_stream1_tc_0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure post request address and data masks */
  if (HAL_MDMA_ConfigPostRequestMask(&hmdma_mdma_channel4_dma1_stream1_tc_0, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure MDMA channel MDMA_Channel5 */
  /* Configure MDMA request hmdma_mdma_channel5_dma1_stream2_tc_0 on MDMA_Channel5 */
  hmdma_mdma_channel5_dma1_stream2_tc_0.Instance = MDMA_Channel5;
  hmdma_mdma_channel5_dma1_stream2_tc_0.Init.Request = MDMA_REQUEST_DMA1_Stream2_TC;
  hmdma_mdma_channel5_dma1_stream2_tc_0.Init.TransferTriggerMode = MDMA_BLOCK_TRANSFER;
  hmdma_mdma_channel5_dma1_stream2_tc_0.Init.Priority = MDMA_PRIORITY_LOW;
  hmdma_mdma_channel5_dma1_stream2_tc_0.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
  hmdma_mdma_channel5_dma1_stream2_tc_0.Init.SourceInc = MDMA_SRC_INC_BYTE;
  hmdma_mdma_channel5_dma1_stream2_tc_0.Init.DestinationInc = MDMA_DEST_INC_BYTE;
  hmdma_mdma_channel5_dma1_stream2_tc_0.Init.SourceDataSize = MDMA_SRC_DATASIZE_BYTE;
  hmdma_mdma_channel5_dma1_stream2_tc_0.Init.DestDataSize = MDMA_DEST_DATASIZE_BYTE;
  hmdma_mdma_channel5_dma1_stream2_tc_0.Init.DataAlignment = MDMA_DATAALIGN_PACKENABLE;
  hmdma_mdma_channel5_dma1_stream2_tc_0.Init.BufferTransferLength = 128;
  hmdma_mdma_channel5_dma1_stream2_tc_0.Init.SourceBurst = MDMA_SOURCE_BURST_SINGLE;
  hmdma_mdma_channel5_dma1_stream2_tc_0.Init.DestBurst = MDMA_DEST_BURST_SINGLE;
  hmdma_mdma_channel5_dma1_stream2_tc_0.Init.SourceBlockAddressOffset = 0;
  hmdma_mdma_channel5_dma1_stream2_tc_0.Init.DestBlockAddressOffset = 0;
  if (HAL_MDMA_Init(&hmdma_mdma_channel5_dma1_stream2_tc_0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure post request address and data masks */
  if (HAL_MDMA_ConfigPostRequestMask(&hmdma_mdma_channel5_dma1_stream2_tc_0, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure MDMA channel MDMA_Channel6 */
  /* Configure MDMA request hmdma_mdma_channel6_dma1_stream3_tc_0 on MDMA_Channel6 */
  hmdma_mdma_channel6_dma1_stream3_tc_0.Instance = MDMA_Channel6;
  hmdma_mdma_channel6_dma1_stream3_tc_0.Init.Request = MDMA_REQUEST_DMA1_Stream3_TC;
  hmdma_mdma_channel6_dma1_stream3_tc_0.Init.TransferTriggerMode = MDMA_BLOCK_TRANSFER;
  hmdma_mdma_channel6_dma1_stream3_tc_0.Init.Priority = MDMA_PRIORITY_LOW;
  hmdma_mdma_channel6_dma1_stream3_tc_0.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
  hmdma_mdma_channel6_dma1_stream3_tc_0.Init.SourceInc = MDMA_SRC_INC_BYTE;
  hmdma_mdma_channel6_dma1_stream3_tc_0.Init.DestinationInc = MDMA_DEST_INC_BYTE;
  hmdma_mdma_channel6_dma1_stream3_tc_0.Init.SourceDataSize = MDMA_SRC_DATASIZE_BYTE;
  hmdma_mdma_channel6_dma1_stream3_tc_0.Init.DestDataSize = MDMA_DEST_DATASIZE_BYTE;
  hmdma_mdma_channel6_dma1_stream3_tc_0.Init.DataAlignment = MDMA_DATAALIGN_PACKENABLE;
  hmdma_mdma_channel6_dma1_stream3_tc_0.Init.BufferTransferLength = 128;
  hmdma_mdma_channel6_dma1_stream3_tc_0.Init.SourceBurst = MDMA_SOURCE_BURST_SINGLE;
  hmdma_mdma_channel6_dma1_stream3_tc_0.Init.DestBurst = MDMA_DEST_BURST_SINGLE;
  hmdma_mdma_channel6_dma1_stream3_tc_0.Init.SourceBlockAddressOffset = 0;
  hmdma_mdma_channel6_dma1_stream3_tc_0.Init.DestBlockAddressOffset = 0;
  if (HAL_MDMA_Init(&hmdma_mdma_channel6_dma1_stream3_tc_0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure post request address and data masks */
  if (HAL_MDMA_ConfigPostRequestMask(&hmdma_mdma_channel6_dma1_stream3_tc_0, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure MDMA channel MDMA_Channel7 */
  /* Configure MDMA request hmdma_mdma_channel7_dma2_stream1_tc_0 on MDMA_Channel7 */
  hmdma_mdma_channel7_dma2_stream1_tc_0.Instance = MDMA_Channel7;
  hmdma_mdma_channel7_dma2_stream1_tc_0.Init.Request = MDMA_REQUEST_DMA2_Stream1_TC;
  hmdma_mdma_channel7_dma2_stream1_tc_0.Init.TransferTriggerMode = MDMA_BLOCK_TRANSFER;
  hmdma_mdma_channel7_dma2_stream1_tc_0.Init.Priority = MDMA_PRIORITY_VERY_HIGH;
  hmdma_mdma_channel7_dma2_stream1_tc_0.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
  hmdma_mdma_channel7_dma2_stream1_tc_0.Init.SourceInc = MDMA_SRC_INC_BYTE;
  hmdma_mdma_channel7_dma2_stream1_tc_0.Init.DestinationInc = MDMA_DEST_INC_BYTE;
  hmdma_mdma_channel7_dma2_stream1_tc_0.Init.SourceDataSize = MDMA_SRC_DATASIZE_BYTE;
  hmdma_mdma_channel7_dma2_stream1_tc_0.Init.DestDataSize = MDMA_DEST_DATASIZE_BYTE;
  hmdma_mdma_channel7_dma2_stream1_tc_0.Init.DataAlignment = MDMA_DATAALIGN_PACKENABLE;
  hmdma_mdma_channel7_dma2_stream1_tc_0.Init.BufferTransferLength = 128;
  hmdma_mdma_channel7_dma2_stream1_tc_0.Init.SourceBurst = MDMA_SOURCE_BURST_SINGLE;
  hmdma_mdma_channel7_dma2_stream1_tc_0.Init.DestBurst = MDMA_DEST_BURST_SINGLE;
  hmdma_mdma_channel7_dma2_stream1_tc_0.Init.SourceBlockAddressOffset = 0;
  hmdma_mdma_channel7_dma2_stream1_tc_0.Init.DestBlockAddressOffset = 0;
  if (HAL_MDMA_Init(&hmdma_mdma_channel7_dma2_stream1_tc_0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure post request address and data masks */
  if (HAL_MDMA_ConfigPostRequestMask(&hmdma_mdma_channel7_dma2_stream1_tc_0, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure MDMA channel MDMA_Channel8 */
  /* Configure MDMA request hmdma_mdma_channel8_dma2_stream4_tc_0 on MDMA_Channel8 */
  hmdma_mdma_channel8_dma2_stream4_tc_0.Instance = MDMA_Channel8;
  hmdma_mdma_channel8_dma2_stream4_tc_0.Init.Request = MDMA_REQUEST_DMA2_Stream4_TC;
  hmdma_mdma_channel8_dma2_stream4_tc_0.Init.TransferTriggerMode = MDMA_BLOCK_TRANSFER;
  hmdma_mdma_channel8_dma2_stream4_tc_0.Init.Priority = MDMA_PRIORITY_HIGH;
  hmdma_mdma_channel8_dma2_stream4_tc_0.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
  hmdma_mdma_channel8_dma2_stream4_tc_0.Init.SourceInc = MDMA_SRC_INC_BYTE;
  hmdma_mdma_channel8_dma2_stream4_tc_0.Init.DestinationInc = MDMA_DEST_INC_BYTE;
  hmdma_mdma_channel8_dma2_stream4_tc_0.Init.SourceDataSize = MDMA_SRC_DATASIZE_BYTE;
  hmdma_mdma_channel8_dma2_stream4_tc_0.Init.DestDataSize = MDMA_DEST_DATASIZE_BYTE;
  hmdma_mdma_channel8_dma2_stream4_tc_0.Init.DataAlignment = MDMA_DATAALIGN_PACKENABLE;
  hmdma_mdma_channel8_dma2_stream4_tc_0.Init.BufferTransferLength = 128;
  hmdma_mdma_channel8_dma2_stream4_tc_0.Init.SourceBurst = MDMA_SOURCE_BURST_SINGLE;
  hmdma_mdma_channel8_dma2_stream4_tc_0.Init.DestBurst = MDMA_DEST_BURST_SINGLE;
  hmdma_mdma_channel8_dma2_stream4_tc_0.Init.SourceBlockAddressOffset = 0;
  hmdma_mdma_channel8_dma2_stream4_tc_0.Init.DestBlockAddressOffset = 0;
  if (HAL_MDMA_Init(&hmdma_mdma_channel8_dma2_stream4_tc_0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure post request address and data masks */
  if (HAL_MDMA_ConfigPostRequestMask(&hmdma_mdma_channel8_dma2_stream4_tc_0, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure MDMA channel MDMA_Channel9 */
  /* Configure MDMA request hmdma_mdma_channel9_dma1_stream6_tc_0 on MDMA_Channel9 */
  hmdma_mdma_channel9_dma1_stream6_tc_0.Instance = MDMA_Channel9;
  hmdma_mdma_channel9_dma1_stream6_tc_0.Init.Request = MDMA_REQUEST_DMA1_Stream6_TC;
  hmdma_mdma_channel9_dma1_stream6_tc_0.Init.TransferTriggerMode = MDMA_BLOCK_TRANSFER;
  hmdma_mdma_channel9_dma1_stream6_tc_0.Init.Priority = MDMA_PRIORITY_LOW;
  hmdma_mdma_channel9_dma1_stream6_tc_0.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
  hmdma_mdma_channel9_dma1_stream6_tc_0.Init.SourceInc = MDMA_SRC_INC_BYTE;
  hmdma_mdma_channel9_dma1_stream6_tc_0.Init.DestinationInc = MDMA_DEST_INC_BYTE;
  hmdma_mdma_channel9_dma1_stream6_tc_0.Init.SourceDataSize = MDMA_SRC_DATASIZE_BYTE;
  hmdma_mdma_channel9_dma1_stream6_tc_0.Init.DestDataSize = MDMA_DEST_DATASIZE_BYTE;
  hmdma_mdma_channel9_dma1_stream6_tc_0.Init.DataAlignment = MDMA_DATAALIGN_PACKENABLE;
  hmdma_mdma_channel9_dma1_stream6_tc_0.Init.BufferTransferLength = 128;
  hmdma_mdma_channel9_dma1_stream6_tc_0.Init.SourceBurst = MDMA_SOURCE_BURST_SINGLE;
  hmdma_mdma_channel9_dma1_stream6_tc_0.Init.DestBurst = MDMA_DEST_BURST_SINGLE;
  hmdma_mdma_channel9_dma1_stream6_tc_0.Init.SourceBlockAddressOffset = 0;
  hmdma_mdma_channel9_dma1_stream6_tc_0.Init.DestBlockAddressOffset = 0;
  if (HAL_MDMA_Init(&hmdma_mdma_channel9_dma1_stream6_tc_0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure post request address and data masks */
  if (HAL_MDMA_ConfigPostRequestMask(&hmdma_mdma_channel9_dma1_stream6_tc_0, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure MDMA channel MDMA_Channel10 */
  /* Configure MDMA request hmdma_mdma_channel10_dma1_stream7_tc_0 on MDMA_Channel10 */
  hmdma_mdma_channel10_dma1_stream7_tc_0.Instance = MDMA_Channel10;
  hmdma_mdma_channel10_dma1_stream7_tc_0.Init.Request = MDMA_REQUEST_DMA1_Stream7_TC;
  hmdma_mdma_channel10_dma1_stream7_tc_0.Init.TransferTriggerMode = MDMA_BLOCK_TRANSFER;
  hmdma_mdma_channel10_dma1_stream7_tc_0.Init.Priority = MDMA_PRIORITY_LOW;
  hmdma_mdma_channel10_dma1_stream7_tc_0.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
  hmdma_mdma_channel10_dma1_stream7_tc_0.Init.SourceInc = MDMA_SRC_INC_BYTE;
  hmdma_mdma_channel10_dma1_stream7_tc_0.Init.DestinationInc = MDMA_DEST_INC_BYTE;
  hmdma_mdma_channel10_dma1_stream7_tc_0.Init.SourceDataSize = MDMA_SRC_DATASIZE_BYTE;
  hmdma_mdma_channel10_dma1_stream7_tc_0.Init.DestDataSize = MDMA_DEST_DATASIZE_BYTE;
  hmdma_mdma_channel10_dma1_stream7_tc_0.Init.DataAlignment = MDMA_DATAALIGN_PACKENABLE;
  hmdma_mdma_channel10_dma1_stream7_tc_0.Init.BufferTransferLength = 128;
  hmdma_mdma_channel10_dma1_stream7_tc_0.Init.SourceBurst = MDMA_SOURCE_BURST_SINGLE;
  hmdma_mdma_channel10_dma1_stream7_tc_0.Init.DestBurst = MDMA_DEST_BURST_SINGLE;
  hmdma_mdma_channel10_dma1_stream7_tc_0.Init.SourceBlockAddressOffset = 0;
  hmdma_mdma_channel10_dma1_stream7_tc_0.Init.DestBlockAddressOffset = 0;
  if (HAL_MDMA_Init(&hmdma_mdma_channel10_dma1_stream7_tc_0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure post request address and data masks */
  if (HAL_MDMA_ConfigPostRequestMask(&hmdma_mdma_channel10_dma1_stream7_tc_0, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure MDMA channel MDMA_Channel11 */
  /* Configure MDMA request hmdma_mdma_channel11_dma2_stream0_tc_0 on MDMA_Channel11 */
  hmdma_mdma_channel11_dma2_stream0_tc_0.Instance = MDMA_Channel11;
  hmdma_mdma_channel11_dma2_stream0_tc_0.Init.Request = MDMA_REQUEST_DMA2_Stream0_TC;
  hmdma_mdma_channel11_dma2_stream0_tc_0.Init.TransferTriggerMode = MDMA_BLOCK_TRANSFER;
  hmdma_mdma_channel11_dma2_stream0_tc_0.Init.Priority = MDMA_PRIORITY_LOW;
  hmdma_mdma_channel11_dma2_stream0_tc_0.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
  hmdma_mdma_channel11_dma2_stream0_tc_0.Init.SourceInc = MDMA_SRC_INC_BYTE;
  hmdma_mdma_channel11_dma2_stream0_tc_0.Init.DestinationInc = MDMA_DEST_INC_BYTE;
  hmdma_mdma_channel11_dma2_stream0_tc_0.Init.SourceDataSize = MDMA_SRC_DATASIZE_BYTE;
  hmdma_mdma_channel11_dma2_stream0_tc_0.Init.DestDataSize = MDMA_DEST_DATASIZE_BYTE;
  hmdma_mdma_channel11_dma2_stream0_tc_0.Init.DataAlignment = MDMA_DATAALIGN_PACKENABLE;
  hmdma_mdma_channel11_dma2_stream0_tc_0.Init.BufferTransferLength = 128;
  hmdma_mdma_channel11_dma2_stream0_tc_0.Init.SourceBurst = MDMA_SOURCE_BURST_SINGLE;
  hmdma_mdma_channel11_dma2_stream0_tc_0.Init.DestBurst = MDMA_DEST_BURST_SINGLE;
  hmdma_mdma_channel11_dma2_stream0_tc_0.Init.SourceBlockAddressOffset = 0;
  hmdma_mdma_channel11_dma2_stream0_tc_0.Init.DestBlockAddressOffset = 0;
  if (HAL_MDMA_Init(&hmdma_mdma_channel11_dma2_stream0_tc_0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure post request address and data masks */
  if (HAL_MDMA_ConfigPostRequestMask(&hmdma_mdma_channel11_dma2_stream0_tc_0, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure MDMA channel MDMA_Channel12 */
  /* Configure MDMA request hmdma_mdma_channel12_dma2_stream2_tc_0 on MDMA_Channel12 */
  hmdma_mdma_channel12_dma2_stream2_tc_0.Instance = MDMA_Channel12;
  hmdma_mdma_channel12_dma2_stream2_tc_0.Init.Request = MDMA_REQUEST_DMA2_Stream2_TC;
  hmdma_mdma_channel12_dma2_stream2_tc_0.Init.TransferTriggerMode = MDMA_BLOCK_TRANSFER;
  hmdma_mdma_channel12_dma2_stream2_tc_0.Init.Priority = MDMA_PRIORITY_LOW;
  hmdma_mdma_channel12_dma2_stream2_tc_0.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
  hmdma_mdma_channel12_dma2_stream2_tc_0.Init.SourceInc = MDMA_SRC_INC_BYTE;
  hmdma_mdma_channel12_dma2_stream2_tc_0.Init.DestinationInc = MDMA_DEST_INC_BYTE;
  hmdma_mdma_channel12_dma2_stream2_tc_0.Init.SourceDataSize = MDMA_SRC_DATASIZE_BYTE;
  hmdma_mdma_channel12_dma2_stream2_tc_0.Init.DestDataSize = MDMA_DEST_DATASIZE_BYTE;
  hmdma_mdma_channel12_dma2_stream2_tc_0.Init.DataAlignment = MDMA_DATAALIGN_PACKENABLE;
  hmdma_mdma_channel12_dma2_stream2_tc_0.Init.BufferTransferLength = 128;
  hmdma_mdma_channel12_dma2_stream2_tc_0.Init.SourceBurst = MDMA_SOURCE_BURST_SINGLE;
  hmdma_mdma_channel12_dma2_stream2_tc_0.Init.DestBurst = MDMA_DEST_BURST_SINGLE;
  hmdma_mdma_channel12_dma2_stream2_tc_0.Init.SourceBlockAddressOffset = 0;
  hmdma_mdma_channel12_dma2_stream2_tc_0.Init.DestBlockAddressOffset = 0;
  if (HAL_MDMA_Init(&hmdma_mdma_channel12_dma2_stream2_tc_0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure post request address and data masks */
  if (HAL_MDMA_ConfigPostRequestMask(&hmdma_mdma_channel12_dma2_stream2_tc_0, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure MDMA channel MDMA_Channel13 */
  /* Configure MDMA request hmdma_mdma_channel13_dma2_stream3_tc_0 on MDMA_Channel13 */
  hmdma_mdma_channel13_dma2_stream3_tc_0.Instance = MDMA_Channel13;
  hmdma_mdma_channel13_dma2_stream3_tc_0.Init.Request = MDMA_REQUEST_DMA2_Stream3_TC;
  hmdma_mdma_channel13_dma2_stream3_tc_0.Init.TransferTriggerMode = MDMA_BLOCK_TRANSFER;
  hmdma_mdma_channel13_dma2_stream3_tc_0.Init.Priority = MDMA_PRIORITY_LOW;
  hmdma_mdma_channel13_dma2_stream3_tc_0.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
  hmdma_mdma_channel13_dma2_stream3_tc_0.Init.SourceInc = MDMA_SRC_INC_BYTE;
  hmdma_mdma_channel13_dma2_stream3_tc_0.Init.DestinationInc = MDMA_DEST_INC_BYTE;
  hmdma_mdma_channel13_dma2_stream3_tc_0.Init.SourceDataSize = MDMA_SRC_DATASIZE_BYTE;
  hmdma_mdma_channel13_dma2_stream3_tc_0.Init.DestDataSize = MDMA_DEST_DATASIZE_BYTE;
  hmdma_mdma_channel13_dma2_stream3_tc_0.Init.DataAlignment = MDMA_DATAALIGN_PACKENABLE;
  hmdma_mdma_channel13_dma2_stream3_tc_0.Init.BufferTransferLength = 32;
  hmdma_mdma_channel13_dma2_stream3_tc_0.Init.SourceBurst = MDMA_SOURCE_BURST_SINGLE;
  hmdma_mdma_channel13_dma2_stream3_tc_0.Init.DestBurst = MDMA_DEST_BURST_SINGLE;
  hmdma_mdma_channel13_dma2_stream3_tc_0.Init.SourceBlockAddressOffset = 0;
  hmdma_mdma_channel13_dma2_stream3_tc_0.Init.DestBlockAddressOffset = 0;
  if (HAL_MDMA_Init(&hmdma_mdma_channel13_dma2_stream3_tc_0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure post request address and data masks */
  if (HAL_MDMA_ConfigPostRequestMask(&hmdma_mdma_channel13_dma2_stream3_tc_0, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure MDMA channel MDMA_Channel14 */
  /* Configure MDMA request hmdma_mdma_channel14_dma2_stream7_tc_0 on MDMA_Channel14 */
  hmdma_mdma_channel14_dma2_stream7_tc_0.Instance = MDMA_Channel14;
  hmdma_mdma_channel14_dma2_stream7_tc_0.Init.Request = MDMA_REQUEST_DMA2_Stream7_TC;
  hmdma_mdma_channel14_dma2_stream7_tc_0.Init.TransferTriggerMode = MDMA_BLOCK_TRANSFER;
  hmdma_mdma_channel14_dma2_stream7_tc_0.Init.Priority = MDMA_PRIORITY_LOW;
  hmdma_mdma_channel14_dma2_stream7_tc_0.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
  hmdma_mdma_channel14_dma2_stream7_tc_0.Init.SourceInc = MDMA_SRC_INC_BYTE;
  hmdma_mdma_channel14_dma2_stream7_tc_0.Init.DestinationInc = MDMA_DEST_INC_BYTE;
  hmdma_mdma_channel14_dma2_stream7_tc_0.Init.SourceDataSize = MDMA_SRC_DATASIZE_BYTE;
  hmdma_mdma_channel14_dma2_stream7_tc_0.Init.DestDataSize = MDMA_DEST_DATASIZE_BYTE;
  hmdma_mdma_channel14_dma2_stream7_tc_0.Init.DataAlignment = MDMA_DATAALIGN_PACKENABLE;
  hmdma_mdma_channel14_dma2_stream7_tc_0.Init.BufferTransferLength = 32;
  hmdma_mdma_channel14_dma2_stream7_tc_0.Init.SourceBurst = MDMA_SOURCE_BURST_SINGLE;
  hmdma_mdma_channel14_dma2_stream7_tc_0.Init.DestBurst = MDMA_DEST_BURST_SINGLE;
  hmdma_mdma_channel14_dma2_stream7_tc_0.Init.SourceBlockAddressOffset = 0;
  hmdma_mdma_channel14_dma2_stream7_tc_0.Init.DestBlockAddressOffset = 0;
  if (HAL_MDMA_Init(&hmdma_mdma_channel14_dma2_stream7_tc_0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure post request address and data masks */
  if (HAL_MDMA_ConfigPostRequestMask(&hmdma_mdma_channel14_dma2_stream7_tc_0, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure MDMA channel MDMA_Channel15 */
  /* Configure MDMA request hmdma_mdma_channel15_dma2_stream5_tc_0 on MDMA_Channel15 */
  hmdma_mdma_channel15_dma2_stream5_tc_0.Instance = MDMA_Channel15;
  hmdma_mdma_channel15_dma2_stream5_tc_0.Init.Request = MDMA_REQUEST_DMA2_Stream5_TC;
  hmdma_mdma_channel15_dma2_stream5_tc_0.Init.TransferTriggerMode = MDMA_BLOCK_TRANSFER;
  hmdma_mdma_channel15_dma2_stream5_tc_0.Init.Priority = MDMA_PRIORITY_MEDIUM;
  hmdma_mdma_channel15_dma2_stream5_tc_0.Init.Endianness = MDMA_LITTLE_ENDIANNESS_PRESERVE;
  hmdma_mdma_channel15_dma2_stream5_tc_0.Init.SourceInc = MDMA_SRC_INC_BYTE;
  hmdma_mdma_channel15_dma2_stream5_tc_0.Init.DestinationInc = MDMA_DEST_INC_BYTE;
  hmdma_mdma_channel15_dma2_stream5_tc_0.Init.SourceDataSize = MDMA_SRC_DATASIZE_BYTE;
  hmdma_mdma_channel15_dma2_stream5_tc_0.Init.DestDataSize = MDMA_DEST_DATASIZE_BYTE;
  hmdma_mdma_channel15_dma2_stream5_tc_0.Init.DataAlignment = MDMA_DATAALIGN_PACKENABLE;
  hmdma_mdma_channel15_dma2_stream5_tc_0.Init.BufferTransferLength = 128;
  hmdma_mdma_channel15_dma2_stream5_tc_0.Init.SourceBurst = MDMA_SOURCE_BURST_SINGLE;
  hmdma_mdma_channel15_dma2_stream5_tc_0.Init.DestBurst = MDMA_DEST_BURST_SINGLE;
  hmdma_mdma_channel15_dma2_stream5_tc_0.Init.SourceBlockAddressOffset = 0;
  hmdma_mdma_channel15_dma2_stream5_tc_0.Init.DestBlockAddressOffset = 0;
  if (HAL_MDMA_Init(&hmdma_mdma_channel15_dma2_stream5_tc_0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure post request address and data masks */
  if (HAL_MDMA_ConfigPostRequestMask(&hmdma_mdma_channel15_dma2_stream5_tc_0, 0, 0) != HAL_OK)
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

