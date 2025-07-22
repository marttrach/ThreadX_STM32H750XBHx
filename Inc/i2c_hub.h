#ifndef I2C_HUB_H
#define I2C_HUB_H

#include "stm32h7xx_hal.h"

/**
 * I2C Hub definitions
 * This file defines the structure and functions for the I2C hub communication.
 */
#define HUB_I2C_ADDR 0x36
#define QUEUE_LEN    10


/* Command targets */
#define HUB_TARGET_UART 0 //Now only UART 8
#define HUB_TARGET_GPIO 1 
#define HUB_TARGET_SPI  2
#define HUB_TARGET_I2C  3 //I2C1 -> UD3 UE3
#define HUB_TARGET_ADC  4
#define HUB_TARGET_PWM  5
#define HUB_TARGET_DAC  6
#define HUB_TARGET_ETH  11
#define HUB_TARGET_WIFI 12 // UART 7
#define HUB_TARGET_CAN  15
#define HUB_TARGET_USB  16
#define HUB_TARGET_SD  17

/**
 * GPIO
 */
/* PIN */
#define HUB_GPIO_PIN_1 'D11'
#define HUB_GPIO_PIN_2 'D12'
#define HUB_GPIO_PIN_3 'D13'
#define HUB_GPIO_PIN_4 'A3'
#define HUB_GPIO_PIN_5 'A4'
/**
 * ADC
 */
/* PIN */
#define HUB_ADC_PIN_0 'C2'
#define HUB_ADC_PIN_1 'A6'
#define HUB_ADC_PIN_2 'C4'
#define HUB_ADC_PIN_3 'B1'
#define HUB_ADC_PIN_4 'C0'
#define HUB_ADC_PIN_5 'C1'
/**
 * PWM
 */
/* PIN */
#define HUB_PWM_PIN_1 'A2'
#define HUB_PWM_PIN_2 'B0'
#define HUB_PWM_PIN_3 'A15'
#define HUB_PWM_PIN_4 'B3'
#define HUB_PWM_PIN_5 'H6'
#define HUB_PWM_PIN_6 'C7'
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


#define MSG_WORDS_CMD  ((sizeof(hub_cmd_t) + sizeof(ULONG) - 1) / sizeof(ULONG))
#define MSG_WORDS_RSP  ((sizeof(hub_rsp_t) + sizeof(ULONG) - 1) / sizeof(ULONG))

/**
 * Error codes for hub responses
 */
#define HUB_RSP_OK 0
#define HUB_RSP_ERR_UNKNOWN_TARGET 1
#define HUB_RSP_ERR_UNKNOWN_CMD 2

typedef struct {
    uint8_t target;      
    uint8_t operation;  
    uint8_t data[16];
} hub_cmd_t;

typedef struct {
    uint8_t status;
    uint8_t data[16];
} hub_rsp_t;

void iot_hub_start(void);

#endif // I2C_HUB_H
