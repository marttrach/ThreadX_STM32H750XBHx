#ifndef IOT_UPGRADE_DEFS_H
#define IOT_UPGRADE_DEFS_H

#include <stdint.h>
#include "iot.h"

#ifndef IOT_EXTERNAL_SD
#define IOT_EXTERNAL_SD        1
#endif

#ifndef IOT_eSD_UPDATE
#define IOT_eSD_UPDATE         1
#endif

#ifndef FW_SIZE_MAX
#define FW_SIZE_MAX            0x1FFFF
#endif

#ifndef IOT_eSD_LOADER_FILENAME
#define IOT_eSD_LOADER_FILENAME "loader.bin"
#endif

#ifndef IOT_eSD_LOADER_FILENAME_BAK
#define IOT_eSD_LOADER_FILENAME_BAK "loader.bak"
#endif

#ifndef FW_START_MAGIC
#define FW_START_MAGIC         0xF0D8C6B0UL /* 'FWMG' placeholder */
#endif

#ifndef FLASH_LOADER_ADDRESS
#define FLASH_LOADER_ADDRESS    0xC0000000UL
#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#endif /* IOT_UPGRADE_DEFS_H */
