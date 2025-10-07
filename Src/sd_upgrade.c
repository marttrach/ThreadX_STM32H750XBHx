#include "sd_upgrade.h"
#include "i2c_hub_filex.h"

#if IOT_EXTERNAL_SD && IOT_eSD_UPDATE
#include <stdint.h>

uint8_t update_loader(char *filename, char *bak_filename)
{
  if (!filename || !bak_filename) {
    DEBUG_DUMP(IOT_LOG_ERR, "update_loader: invalid arguments\r\n");
    return (uint8_t)-1;
  }

  FX_DIR_ENTRY entry;
  UINT fx_status = FX_Stat(filename, &entry);
  if (fx_status != FX_SUCCESS) {
    DEBUG_DUMP(IOT_LOG_ERR, "update_loader: FX_Stat('%s') failed: 0x%02X\r\n", filename, fx_status);
    return (uint8_t)-1;
  }

  ULONG64 file_size = entry.fx_dir_entry_file_size;
  if (file_size == 0U) {
    DEBUG_DUMP(IOT_LOG_ERR, "update_loader: '%s' is empty\r\n", filename);
    return (uint8_t)-1;
  }
  if (file_size > FW_SIZE_MAX) {
    DEBUG_DUMP(IOT_LOG_ERR, "update_loader: '%s' too large (%llu bytes)\r\n", filename, (unsigned long long)file_size);
    return (uint8_t)-1;
  }
  if (file_size > UINT32_MAX) {
    DEBUG_DUMP(IOT_LOG_ERR, "update_loader: '%s' exceeds 32-bit size limit\r\n", filename);
    return (uint8_t)-1;
  }

  uint32_t expected = (uint32_t)file_size;
  uint32_t bytesread = 0U;
  fx_status = FX_ReadFile(filename, (char *)FLASH_LOADER_ADDRESS, expected, &bytesread);
  if (fx_status != FX_SUCCESS) {
    DEBUG_DUMP(IOT_LOG_ERR, "update_loader: FX_ReadFile('%s') failed: 0x%02X\r\n", filename, fx_status);
    return (uint8_t)-1;
  }
  if (bytesread != expected) {
    DEBUG_DUMP(IOT_LOG_ERR, "update_loader: short read (%u/%u bytes)\r\n", bytesread, expected);
    return (uint8_t)-1;
  }

  fx_status = FX_Rename(filename, bak_filename);
  if (fx_status != FX_SUCCESS) {
    DEBUG_DUMP(IOT_LOG_ERR, "update_loader: FX_Rename('%s','%s') failed: 0x%02X\r\n", filename, bak_filename, fx_status);
    return (uint8_t)-1;
  }

  DEBUG_DUMP(IOT_LOG_INFO, "update_loader: programming %u bytes\r\n", expected);
  iot_do_fw_upgrade((uint32_t)FLASH_LOADER_ADDRESS, expected);
  return 0;
}

#endif /* IOT_EXTERNAL_SD && IOT_eSD_UPDATE */
