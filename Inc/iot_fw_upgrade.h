#ifndef IOT_FW_UPGRADE_H
#define IOT_FW_UPGRADE_H

#include <stdint.h>
#include "iot_upgrade_defs.h"
#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

HAL_StatusTypeDef IOT_FLASH_WaitForLastOperation(void);
HAL_StatusTypeDef IOT_HAL_FLASHEx_Erase(void);
HAL_StatusTypeDef IFLASH_Program(uint32_t new_firmware_address, uint32_t new_firmware_len);
void iot_do_fw_upgrade(uint32_t new_firmware_address, uint32_t new_firmware_len);

#ifdef __cplusplus
}
#endif

#endif /* IOT_FW_UPGRADE_H */
