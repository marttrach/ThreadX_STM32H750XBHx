/**
  ******************************************************************************
  * @file   fatfs.c
  * @brief  Code for fatfs applications (ported for current project)
  ******************************************************************************
  */

#include "fatfs.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "sd_diskio.h"
#include "rtc.h"

uint8_t retSD;

static FIL MyFile;
static uint8_t workBuffer[_MAX_SS];

#if IOT_EXTERNAL_SD
char eSDPath[4];
FATFS eSDFatFS;
FIL eSDFile;
#endif

#if IOT_EXTERNAL_SD
int eFS_Open(FIL *fp, char *orig_filename)
{
  if (f_mount(&eSDFatFS, (TCHAR const*)eSDPath, 0) != FR_OK) {
    return -1;
  }
  char filename[256];
  snprintf(filename, sizeof(filename), "%s%s", eSDPath, orig_filename);
  return (f_open(fp, filename, FA_READ) == FR_OK) ? 0 : -1;
}

int eFS_Close(FIL *fp)
{
  (void)fp;
  f_close(&MyFile);
  return 0;
}

int eFS_Rename(char *old_path, char *new_path)
{
  if (f_mount(&eSDFatFS, (TCHAR const*)eSDPath, 0) != FR_OK) {
    return -1;
  }
  char old_filename[256];
  char new_filename[256];
  snprintf(old_filename, sizeof(old_filename), "%s%s", eSDPath, old_path);
  snprintf(new_filename, sizeof(new_filename), "%s%s", eSDPath, new_path);
  (void)f_unlink(new_filename);
  if (f_rename(old_filename, new_filename) == FR_OK) {
    DEBUG_DUMP(IOT_LOG_INFO, "File '%s' rename to '%s'\r\n", old_path, new_path);
    return 0;
  }
  return -1;
}

int eFS_Read(FIL *fp, char *buf, uint32_t size, uint32_t *bytesread)
{
  uint32_t filesize = size;
  if (bytesread) {
    *bytesread = 0U;
  }
  if (filesize == 0U) {
    filesize = f_size(fp);
  }
  DEBUG_DUMP(IOT_LOG_DEBUG, "Read size=%lu\r\n", filesize);
  if (f_read(fp, buf, filesize, (void *)bytesread) == FR_OK && bytesread && *bytesread > 0U) {
    DEBUG_DUMP(IOT_LOG_DEBUG, "fs_read %lu\r\n", (unsigned long)*bytesread);
    return 0;
  }
  return -1;
}

int eFS_Size(FIL *fp)
{
  return (int)f_size(fp);
}


FRESULT eFS_ReadFile(char *orig_filename, char *buf, uint32_t size, uint32_t *bytesread)
{
  FRESULT res; /* FatFs function common result code */
  uint32_t filesize = size;

  *bytesread = 0; //Set to zero at startup
  /* Register the file system object to the FatFs module */
  if(f_mount(&eSDFatFS, (TCHAR const*)eSDPath, 0) == FR_OK) {
    char filename[256];
    sprintf(filename, "%s%s", eSDPath, orig_filename);
    /* Open the text file object with read access */
    if(f_open(&MyFile, filename, FA_READ) == FR_OK) {
      if ( !filesize ) filesize = f_size(&MyFile); 
      /* Read data from the text file */
      res = f_read(&MyFile, buf, filesize, (void *)bytesread);

      if((*bytesread > 0) && (res == FR_OK)) {
        DEBUG_DUMP(IOT_LOG_DEBUG, "fs_read %ld\r\n", *bytesread);
        /* Close the open text file */
        f_close(&MyFile);
        return FR_OK;
      }
      DEBUG_DUMP(IOT_LOG_ERR, "read failed\r\n");
      return FR_DISK_ERR;
    }
    DEBUG_DUMP(IOT_LOG_ERR, "open failed\r\n");
    return FR_NO_FILE;
  }
  DEBUG_DUMP(IOT_LOG_ERR, "mount failed\r\n");
  return FR_NO_FILESYSTEM;
}

FRESULT eFS_WriteFile(char *orig_filename, char *buf, uint32_t size)
{
  uint32_t byteswritten;
  if (f_mount(&eSDFatFS, (TCHAR const*)eSDPath, 0) != FR_OK) {
    DEBUG_DUMP(IOT_LOG_DEBUG, "mount failed\r\n");
    return FR_NO_FILESYSTEM;
  }
  char filename[256];
  snprintf(filename, sizeof(filename), "%s%s", eSDPath, orig_filename);
  if (f_open(&MyFile, filename, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) {
    DEBUG_DUMP(IOT_LOG_DEBUG, "open failed\r\n");
    return FR_WRITE_PROTECTED;
  }
  DEBUG_DUMP(IOT_LOG_DEBUG, "Write '%s', size=%lu\r\n", filename, size);
  if (f_write(&MyFile, buf, size, (void *)&byteswritten) == FR_OK && byteswritten > 0U) {
    DEBUG_DUMP(IOT_LOG_DEBUG, "fs_write %lu\r\n", (unsigned long)byteswritten);
    f_close(&MyFile);
    return FR_OK;
  }
  DEBUG_DUMP(IOT_LOG_DEBUG, "write failed\r\n");
  f_close(&MyFile);
  return FR_DISK_ERR;
}
#endif

void MX_FATFS_Init(void)
{
#if IOT_EXTERNAL_SD
  retSD = FATFS_LinkDriver(&eSD_Driver, eSDPath);
  if (retSD == 0U) {
    DEBUG_DUMP(IOT_LOG_DEBUG, "FATFS LinkDriver to eSD ok\r\n");
    FRESULT res = f_mount(&eSDFatFS, (TCHAR const*)eSDPath, 1);
    if (res != FR_OK) {
      DEBUG_DUMP(IOT_LOG_DEBUG, "eSD fatfs mount failed(%d)\r\n", res);
    } else {
      DEBUG_DUMP(IOT_LOG_DEBUG, "'%s' path mount ok\r\n", eSDPath);
    }
  }
#endif
}

DWORD get_fattime(void)
{
  DWORD fattime = 0U;
  RTC_DateTypeDef date = {0};
  RTC_TimeTypeDef time = {0};
  HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);
  fattime = (((DWORD)date.Year + 20U) << 25) |
            ((DWORD)date.Month << 21) |
            ((DWORD)date.Date << 16) |
            ((DWORD)time.Hours << 11) |
            ((DWORD)time.Minutes << 5) |
            ((DWORD)time.Seconds >> 1);
  return fattime;
}

/**
  * @brief  This function calculates the checksum of the received address.
  * @param  the table where the address is stored and the size of this table.
  * @retval Calculated checksum.
  */
uint8_t Verify_Checksum(uint8_t *T, uint32_t size)
{  
  uint32_t cnt;
  uint8_t checksum;
  checksum = T[0];

  for (cnt=1; cnt<size-1;cnt++)
  {
    checksum = checksum ^ T[cnt];
  }	 
  return checksum;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
