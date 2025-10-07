/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sd_diskio.c
  * @brief   SD Disk I/O driver
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Note: code generation based on sd_diskio_template_bspv1.c v2.1.4 
   as "Use dma template" is disabled. */

/* USER CODE BEGIN firstSection */
/* can be used to modify / undefine following code or add new definitions */
/* USER CODE END firstSection*/

/* Includes ------------------------------------------------------------------*/
#include "ff_gen_drv.h"
#include "sd_diskio.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* use the default SD timout as defined in the platform BSP driver*/
#if defined(SDMMC_DATATIMEOUT)
#define SD_TIMEOUT SDMMC_DATATIMEOUT
#elif defined(SD_DATATIMEOUT)
#define SD_TIMEOUT SD_DATATIMEOUT
#else
#define SD_TIMEOUT 30 * 1000
#endif

#define SD_DEFAULT_BLOCK_SIZE 512

/*
 * Depending on the use case, the SD card initialization could be done at the
 * application level: if it is the case define the flag below to disable
 * the BSP_SD_Init() call in the SD_Initialize() and add a call to 
 * BSP_SD_Init() elsewhere in the application.
 */
/* USER CODE BEGIN disableSDInit */
/* #define DISABLE_SD_INIT */
/* Undefine previous, and then define new one for IOT used (5 seconds, seems too long?!), foxes, 20210421 */
#undef SD_TIMEOUT
#define SD_TIMEOUT  5*1000 /* 5 seconds */
/* USER CODE END disableSDInit */

/* Private variables ---------------------------------------------------------*/
/* Disk status */
static volatile DSTATUS Stat = STA_NOINIT;

/* Private function prototypes -----------------------------------------------*/
static DSTATUS eSD_CheckStatus(BYTE lun);
DSTATUS eSD_initialize (BYTE);
DSTATUS eSD_status (BYTE);
DRESULT eSD_read (BYTE, BYTE*, DWORD, UINT);
#if _USE_WRITE == 1
DRESULT eSD_write (BYTE, const BYTE*, DWORD, UINT);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
DRESULT eSD_ioctl (BYTE, BYTE, void*);
#endif  /* _USE_IOCTL == 1 */

const Diskio_drvTypeDef  eSD_Driver =
{
  eSD_initialize,
  eSD_status,
  eSD_read,
#if  _USE_WRITE == 1
  eSD_write,
#endif /* _USE_WRITE == 1 */

#if  _USE_IOCTL == 1
  eSD_ioctl,
#endif /* _USE_IOCTL == 1 */
};

/* USER CODE BEGIN beforeFunctionSection */
/* can be used to modify / undefine following code or add new code */
/* USER CODE END beforeFunctionSection */

/* Private functions ---------------------------------------------------------*/

static DSTATUS eSD_CheckStatus(BYTE lun)
{
  Stat = STA_NOINIT;

  if(BSP_eSD_GetCardState() == MSD_OK)
  {
    Stat &= ~STA_NOINIT;
  }

  return Stat;
}

/**
  * @brief  Initializes a Drive
  * @param  lun : not used
  * @retval DSTATUS: Operation status
  */
DSTATUS eSD_initialize(BYTE lun)
{
Stat = STA_NOINIT;  

#if !defined(DISABLE_SD_INIT)
  if(BSP_eSD_Init() == MSD_OK)
  {
    Stat = eSD_CheckStatus(lun);
  }

#else
  Stat = eSD_CheckStatus(lun);
#endif

  return Stat;
}

/**
  * @brief  Gets Disk Status
  * @param  lun : not used
  * @retval DSTATUS: Operation status
  */
DSTATUS eSD_status(BYTE lun)
{
  return eSD_CheckStatus(lun);
}

/* USER CODE BEGIN beforeReadSection */
/* can be used to modify previous code / undefine following code / add new code */
/* USER CODE END beforeReadSection */
/**
  * @brief  Reads Sector(s)
  * @param  lun : not used
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
              
DRESULT eSD_read(BYTE lun, BYTE *buff, DWORD sector, UINT count)
{
  DRESULT res = RES_ERROR;

  if(BSP_eSD_ReadBlocks((uint32_t*)buff,
                       (uint32_t) (sector),
                       count, SD_TIMEOUT) == MSD_OK)
  {
    /* wait until the read operation is finished */
    while(BSP_eSD_GetCardState()!= MSD_OK)
    {
    }
    res = RES_OK;
  }

  return res;
}

/* USER CODE BEGIN beforeWriteSection */
/* can be used to modify previous code / undefine following code / add new code */
/* USER CODE END beforeWriteSection */
/**
  * @brief  Writes Sector(s)
  * @param  lun : not used
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
#if _USE_WRITE == 1
              
DRESULT eSD_write(BYTE lun, const BYTE *buff, DWORD sector, UINT count)
{
  DRESULT res = RES_ERROR;

  if(BSP_eSD_WriteBlocks((uint32_t*)buff,
                        (uint32_t)(sector),
                        count, SD_TIMEOUT) == MSD_OK)
  {
	/* wait until the Write operation is finished */
    while(BSP_eSD_GetCardState() != MSD_OK)
    {
    }
    res = RES_OK;
  }

  return res;
}
#endif /* _USE_WRITE == 1 */

/* USER CODE BEGIN beforeIoctlSection */
/* can be used to modify previous code / undefine following code / add new code */
/* USER CODE END beforeIoctlSection */
/**
  * @brief  I/O control operation
  * @param  lun : not used
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
#if _USE_IOCTL == 1
DRESULT eSD_ioctl(BYTE lun, BYTE cmd, void *buff)
{
  DRESULT res = RES_ERROR;
  BSP_SD_CardInfo CardInfo;

  if (Stat & STA_NOINIT) return RES_NOTRDY;

  switch (cmd)
  {
  /* Make sure that no pending write process */
  case CTRL_SYNC :
    res = RES_OK;
    break;

  /* Get number of sectors on the disk (DWORD) */
  case GET_SECTOR_COUNT :
    BSP_eSD_GetCardInfo(&CardInfo);
    *(DWORD*)buff = CardInfo.LogBlockNbr;
    res = RES_OK;
    break;

  /* Get R/W sector size (WORD) */
  case GET_SECTOR_SIZE :
    BSP_eSD_GetCardInfo(&CardInfo);
    *(WORD*)buff = CardInfo.LogBlockSize;
    res = RES_OK;
    break;

  /* Get erase block size in unit of sector (DWORD) */
  case GET_BLOCK_SIZE :
    BSP_eSD_GetCardInfo(&CardInfo);
    *(DWORD*)buff = CardInfo.LogBlockSize / SD_DEFAULT_BLOCK_SIZE;
    res = RES_OK;
    break;

  default:
    res = RES_PARERR;
  }

  return res;
}
#endif /* _USE_IOCTL == 1 */

/* USER CODE BEGIN afterIoctlSection */
/* can be used to modify previous code / undefine following code / add new code */
/* USER CODE END afterIoctlSection */

/* USER CODE BEGIN lastSection */ 
/* can be used to modify / undefine previous code or add new code */
/* USER CODE END lastSection */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

