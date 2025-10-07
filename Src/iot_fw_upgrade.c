#include "iot_fw_upgrade.h"
#include "main.h"
#include <string.h>

extern __IO uint32_t uwTick;

#define FLASH_TIMEOUT_VALUE       50000U
#define FIRMWARE_ADDRESS          0x08000000UL

HAL_StatusTypeDef __attribute__ ((section (".upgrade_section.IOT_FLASH_WaitForLastOperation"))) IOT_FLASH_WaitForLastOperation(void)
{
  uint32_t tickstart = uwTick;
  while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_QW_BANK1)) {
    if ((uwTick - tickstart) > FLASH_TIMEOUT_VALUE) {
      return HAL_TIMEOUT;
    }
  }

  if (((FLASH->SR1 & FLASH_FLAG_ALL_ERRORS_BANK1) & 0x7FFFFFFFU) != 0U) {
    __HAL_FLASH_CLEAR_FLAG(FLASH->SR1 & FLASH_FLAG_ALL_ERRORS_BANK1);
    return HAL_ERROR;
  }

  if (__HAL_FLASH_GET_FLAG_BANK1(FLASH_FLAG_EOP_BANK1)) {
    __HAL_FLASH_CLEAR_FLAG_BANK1(FLASH_FLAG_EOP_BANK1);
  }

  return HAL_OK;
}

HAL_StatusTypeDef __attribute__ ((section (".upgrade_section.IOT_HAL_FLASHEx_Erase"))) IOT_HAL_FLASHEx_Erase(void)
{
  HAL_StatusTypeDef status = IOT_FLASH_WaitForLastOperation();
  if (status == HAL_OK) {
    FLASH->CR1 &= (~FLASH_CR_PSIZE);
    FLASH->CR1 |= FLASH_CR_BER | FLASH_VOLTAGE_RANGE_4;
    FLASH->CR1 |= FLASH_CR_START;
    status = IOT_FLASH_WaitForLastOperation();
    FLASH->CR1 &= (~FLASH_CR_BER);
  }
  return status;
}

HAL_StatusTypeDef __attribute__ ((section (".upgrade_section.IFLASH_Program"))) IFLASH_Program(uint32_t new_firmware_address, uint32_t new_firmware_len)
{
  HAL_StatusTypeDef status = IOT_HAL_FLASHEx_Erase();
  if (status != HAL_OK) {
    return status;
  }

  status = IOT_FLASH_WaitForLastOperation();
  uint32_t len = (new_firmware_len + FLASH_NB_32BITWORD_IN_FLASHWORD - 1U) / FLASH_NB_32BITWORD_IN_FLASHWORD;
  __IO uint32_t *s_data = (__IO uint32_t *)new_firmware_address;
  __IO uint32_t *d_data = (__IO uint32_t *)FIRMWARE_ADDRESS;

  for (uint32_t i = 0; (i < len) && (status == HAL_OK); i++) {
    uint8_t row_index = FLASH_NB_32BITWORD_IN_FLASHWORD;
    SET_BIT(FLASH->CR1, FLASH_CR_PG);
    __ISB();
    __DSB();
    while (row_index--) {
      *d_data++ = *s_data++;
    }
    __ISB();
    __DSB();
    CLEAR_BIT(FLASH->CR1, FLASH_CR_PG);
    status = IOT_FLASH_WaitForLastOperation();
  }

  CLEAR_BIT(FLASH->CR1, FLASH_CR_PG);
  (void)IOT_FLASH_WaitForLastOperation();
  NVIC_SystemReset();
  return status;
}

void iot_do_fw_upgrade(uint32_t new_firmware_address, uint32_t new_firmware_len)
{
  extern char _supgrade_section, _upgrade_section, _eupgrade_section, _sisr_vector, _eisr_vector, _new_vector;
  SCB_DisableICache();
  SCB_DisableDCache();
  memcpy(&_supgrade_section, &_upgrade_section, (size_t)(&_eupgrade_section - &_supgrade_section));
  memcpy(&_new_vector, &_sisr_vector, (size_t)(&_eisr_vector - &_sisr_vector));
  HAL_FLASH_Unlock();
  __disable_irq();
  SCB->VTOR = (uint32_t)&_new_vector;
  DEBUG_DUMP(IOT_LOG_INFO, "!!!Upgrade internal firmware!!!\r\n");
  if (IFLASH_Program(new_firmware_address, new_firmware_len) != HAL_OK) {
    DEBUG_DUMP(IOT_LOG_ERR, "Error upgrade?\r\n");
  }
}
