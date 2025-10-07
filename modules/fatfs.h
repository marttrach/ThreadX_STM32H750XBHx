#ifndef FATFS_USER_H
#define FATFS_USER_H

#include "iot_upgrade_defs.h"

#if IOT_EXTERNAL_SD || 1
#include "ff.h"
#include "ff_gen_drv.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

extern uint8_t retSD;
/*
extern char SDPath[4];
extern FATFS SDFatFS;
extern FIL SDFile;
*/

#if IOT_EXTERNAL_SD
extern char eSDPath[4];
extern FATFS eSDFatFS;
extern FIL eSDFile;
#endif

#if IOT_EXTERNAL_SD
int eFS_Open(FIL *fp, char *orig_filename);
int eFS_Close(FIL *fp);
int eFS_Rename(char *old_path, char *new_path);
int eFS_Read(FIL *fp, char *buf, uint32_t size, uint32_t *bytesread);
int eFS_Size(FIL *fp);
FRESULT eFS_ReadFile(char *orig_filename, char *buf, uint32_t size, uint32_t *bytesread);
FRESULT eFS_WriteFile(char *orig_filename, char *buf, uint32_t size);
#endif

#ifdef __cplusplus
}
#endif

#endif /* FATFS_USER_H */
