#ifndef I2C_HUB_UART_H
#define I2C_HUB_UART_H

#include "usart.h"
#include "i2c_hub.h"

/* 2 x ( Rx + Tx )= 20480bytes => 96-20 = 76K remain */
#define UART_TX_BUF_SZ  (2048U)
#define UART_RX_BUF_SZ  (2048U)  
_Static_assert((UART_RX_BUF_SZ & (UART_RX_BUF_SZ - 1U)) == 0, "UART_RX_BUF_SZ must be power of two");
_Static_assert((UART_TX_BUF_SZ & (UART_TX_BUF_SZ - 1U)) == 0, "UART_TX_BUF_SZ must be power of two");

typedef struct {
    volatile uint32_t busy;
    volatile uint32_t write_ptr;
    volatile uint32_t read_ptr;
    uint8_t buf[UART_TX_BUF_SZ] __attribute__((aligned(32)));
} iot_uart_tx_ring_t;

typedef struct {
    volatile uint32_t write_ptr;
    volatile uint32_t read_ptr;
    uint8_t  buf[UART_RX_BUF_SZ] __attribute__((aligned(32)));
} iot_uart_rx_ring_t;

/* RX */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
void uart7_rx_dma_start(void);
uint32_t uart7_rx_available(void);
uint32_t uart7_rx_read(uint8_t *dst, uint32_t len);
void uart8_rx_dma_start(void);
uint32_t uart8_rx_available(void);
uint32_t uart8_rx_read(uint8_t *dst, uint32_t len);

/* TX */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);

void _uart7_tx_dma_done(void);
void _uart7_tx_dma_touch(void);
void iot_uart7_tx_write(uint8_t *ptr, uint32_t len);
uint8_t *uart7_hub_helper(uint8_t *dst_ptr, hub_cmd_t *cmd, uint16_t avail);

void _uart8_tx_dma_done(void);
void _uart8_tx_dma_touch(void);
void iot_uart8_tx_write(uint8_t *ptr, uint32_t len);
uint8_t *uart8_hub_helper(uint8_t *dst_ptr, hub_cmd_t *cmd, uint16_t avail);

void iot_uart_init(void);

#endif /* I2C_HUB_UART_H */