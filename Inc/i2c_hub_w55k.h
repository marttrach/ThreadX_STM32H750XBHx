#ifndef W5500_PORT_H
#define W5500_PORT_H

#include <stdint.h>
#include <stddef.h>
#include "stm32h7xx_hal.h"
#include "tx_api.h"
#include "iot.h"

#ifndef W5500_DEF_MAC0
#define W5500_DEF_MAC0  0x02,0x08,0xDC,0x00,0x00,0x02
#endif

#ifndef W5500_DEF_IP
#define W5500_DEF_IP    10,0,0,211
#endif

#ifndef W5500_DEF_MASK
#define W5500_DEF_MASK  255,255,255,0
#endif

#ifndef W5500_DEF_GW
#define W5500_DEF_GW    10,0,0,1
#endif

#ifndef W5500_DEF_DNS
#define W5500_DEF_DNS   10,0,0,1
#endif

#ifndef W5500_SPI_TIMEOUT_MS
#define W5500_SPI_TIMEOUT_MS  20U
#endif

#define MS_TO_TICKS(ms)  ( ((ms) * TX_TIMER_TICKS_PER_SECOND + 999) / 1000 )

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;  uint16_t cs_pin;
    GPIO_TypeDef *rst_port; uint16_t rst_pin;
    GPIO_TypeDef *int_port; uint16_t int_pin;

    UINT mutex_inherit;
} w5500_port_cfg_t;

typedef struct {
    uint8_t mac[6];
    uint8_t ip[4];
    uint8_t mask[4];
    uint8_t gw[4];
    uint8_t dns[4];
    uint8_t use_dhcp;  /* 0 = static, 1 = dhcp (need to add in cmakefilelist)*/
} w5500_net_cfg_t;

int  w5500_port_init(const w5500_port_cfg_t *pcfg);
int  w5500_bringup(const w5500_net_cfg_t *ncfg);  /* ram/PHY link */

void w5500_lock(void);
void w5500_unlock(void);

void w5500_hw_reset(void);
void w5500_delay_ms(uint32_t ms);

/* get link status */
int  w5500_is_link_up(void);

int  w5500_spi_set_prescaler(uint32_t prescaler);

#ifdef __cplusplus
}
#endif
#endif /* W5500_PORT_H */
