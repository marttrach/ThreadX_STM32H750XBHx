#ifndef I2C_HUB_H
#define I2C_HUB_H

#include "stm32h7xx_hal.h"
#include "app_threadx.h"
#define ALIGN32(x)   (((x) + 31U) & ~31U)
/**
 * I2C Hub definitions
 * This file defines the structure and functions for the I2C hub communication.
 */
#define HUB_I2C_ADDR 0x36
#define QUEUE_LEN    100
#define HUB_SDRAM_BASE 0xC0000000
#define HUB_SDRAM_SIZE 0x02000000

/* Command targets */
#define HUB_TARGET_UART 0 //Now only UART 8
#define HUB_TARGET_GPIO 1 
#define HUB_TARGET_SPI  2
#define HUB_TARGET_I2C  3 //I2C1 -> UD3 UE3
#define HUB_TARGET_ADC  4
#define HUB_TARGET_PWM  5
#define HUB_TARGET_DAC  6
#define HUB_TARGET_WIFI 12 // UART 7
#define HUB_TARGET_CAN  15
// #define HUB_TARGET_USB  16
#define HUB_TARGET_SD  17

/* CONTROL */
#define HUB_IOCT_CFG_SET 0
#define HUB_IOCT_CFG_GET 1
#define HUB_IOCT_CFG_TOGGLE 2
/* CONFIGURATION */
#define HUB_IOCT_CFG_NONE 0
#define HUB_IOCT_CFG_INPUT 1
#define HUB_IOCT_CFG_OUTPUT 2
#define HUB_IOCT_CFG_ANALOG 3
#define HUB_IOCT_CFG_AF 4
#define HUB_IOCT_CFG_PULLUP 5
#define HUB_IOCT_CFG_PULLDOWN 6
#define HUB_IOCT_CFG_OD 7

/**
 * Error codes for hub responses
 */
typedef enum {
    HUB_RSP_OK = 0,
    HUB_RSP_ERR_UNKNOWN_TARGET,
    HUB_RSP_ERR_UNKNOWN_CMD,
    HUB_RSP_ERR_CRC,
    HUB_RSP_ERR_NONE
} hub_rsp_status_t;

/* Generic operations used by the state machine */
typedef enum {
    HUB_OP_NOP = 0,
    HUB_OP_WRITE,
    HUB_OP_READ,
    HUB_OP_CONFIG
} hub_operation_t;

/* CMD */
typedef struct __attribute__((packed, aligned(32))) {
    uint8_t  target;
    hub_operation_t  operation;
    uint16_t len;          /* payload len (byte) */
    uint32_t data_addr;    /* start frame payload in sdram */
    uint32_t crc;          /* crc32*/
} hub_cmd_t;

/* RSP */
typedef struct __attribute__((packed, aligned(32))) {
    hub_rsp_status_t  status;
    uint8_t  reserved;
    uint16_t len;
    uint32_t data_addr;
    uint32_t crc;
} hub_rsp_t;

/* PIN PACK */
typedef struct __attribute__((packed)){
    uint16_t pin_code;        /* Macro HUB_ENC_PIN() init */
    uint8_t  cfg;             /* HUB_IOCT_CFG_*       */
    uint8_t  af;              /* Alternate Function (0 = no change) */
} hub_cfg_payload_t;

#define MSG_WORDS_CMD  ((sizeof(hub_cmd_t) + sizeof(ULONG) - 1) / sizeof(ULONG))
#define MSG_WORDS_RSP  ((sizeof(hub_rsp_t) + sizeof(ULONG) - 1) / sizeof(ULONG))

void iot_hub_start(void);

#endif // I2C_HUB_H
