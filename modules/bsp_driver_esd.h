/**
 ******************************************************************************
  * @file    bsp_driver_esd.h (based on stm32h743i_eval_sd.h)
  * @brief   This file contains the common defines and functions prototypes for 
  *          the bsp_driver_esd.c driver.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32H7_ESD_H
#define __STM32H7_ESD_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Exported types --------------------------------------------------------*/ 
/** 
  * @brief SD Card information structure 
  */
#define BSP_SD_CardInfo HAL_SD_CardInfoTypeDef

/* Exported constants --------------------------------------------------------*/ 
/**
  * @brief  SD status structure definition  
  */     
#define   MSD_OK                        ((uint8_t)0x00)
#define   MSD_ERROR                     ((uint8_t)0x01)
#define   MSD_ERROR_SD_NOT_PRESENT      ((uint8_t)0x02)

/** 
  * @brief  SD transfer state definition  
  */     
#define   SD_TRANSFER_OK                ((uint8_t)0x00)
#define   SD_TRANSFER_BUSY              ((uint8_t)0x01)

#define SD_PRESENT               ((uint8_t)0x01)
#define SD_NOT_PRESENT           ((uint8_t)0x00)
#define SD_DATATIMEOUT           ((uint32_t)100000000)
#define SD_DETECT_PIN         GPIO_PIN_14
#define SD_DETECT_GPIO_PORT   GPIOG

/* USER CODE BEGIN BSP_H_CODE */
#define SD_DetectIRQHandler()             HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8)

/* Exported functions --------------------------------------------------------*/
uint8_t BSP_eSD_Init(void);
uint8_t BSP_eSD_ITConfig(void);
uint8_t BSP_eSD_ReadBlocks(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks, uint32_t Timeout);
uint8_t BSP_eSD_WriteBlocks(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks, uint32_t Timeout);
uint8_t BSP_eSD_ReadBlocks_DMA(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks);
uint8_t BSP_eSD_WriteBlocks_DMA(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks);
uint8_t BSP_eSD_Erase(uint32_t StartAddr, uint32_t EndAddr);
uint8_t BSP_eSD_GetCardState(void);
void    BSP_eSD_GetCardInfo(BSP_SD_CardInfo *CardInfo);
uint8_t BSP_eSD_IsDetected(void);
/* These functions can be modified in case the current settings (e.g. DMA stream)
   need to be changed for specific application needs */
void    BSP_SD_AbortCallback(void);
void    BSP_SD_WriteCpltCallback(void);
void    BSP_SD_ReadCpltCallback(void);
uint8_t	BSP_PlatformIsDetected(void);
/* USER CODE END BSP_H_CODE */ 

#ifdef __cplusplus
}
#endif

#endif /* __STM32H7_ESD_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
