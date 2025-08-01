#ifndef I2C_HUB_MDMA_H
#define I2C_HUB_MDMA_H

#pragma once
#include "i2c_hub.h"
#include "mdma.h"

#define HUB_SDRAM_BASE 0xC0000000
#define HUB_SDRAM_SIZE 0x02000000

#define HUB_BUF_SLAVE_I2C   4096
#define HUB_BUF_SZ_UART   4096
#define HUB_BUF_SZ_SPI    4096
#define HUB_BUF_SZ_I2C    4096
#define HUB_BUF_SZ_WIFI  8192
#define HUB_BUF_SZ_MEM    8192

static inline void *hub_sdram_alloc(uint32_t *offset, size_t n)
{
    void *p = (void *)(HUB_SDRAM_BASE + *offset);
    *offset = ALIGN32(*offset + n);
    return p;
}


#endif // I2C_HUB_MDMA_H