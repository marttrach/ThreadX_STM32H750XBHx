#ifndef SD_UPGRADE_H
#define SD_UPGRADE_H

#include "iot_upgrade_defs.h"
#include "iot_fw_upgrade.h"

#ifdef __cplusplus
extern "C" {
#endif

// int check_loader_from_memory(uint8_t *buffer, uint8_t *decrypt_buffer, uint32_t bytesread);
uint8_t update_loader(char *filename, char *bak_filename);

#ifdef __cplusplus
}
#endif

#endif /* SD_UPGRADE_H */
