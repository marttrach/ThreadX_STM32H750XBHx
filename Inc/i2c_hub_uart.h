#ifndef I2C_HUB_UART_H
#define I2C_HUB_UART_H

#include "usart.h"

#define UART_TX_BUF_SZ  (1024U)
#define UART_RX_BUF_SZ  (1024U)  

typedef struct __attribute__((packed, aligned(4))) {
    uint32_t busy;
    uint8_t buf[UART_TX_BUF_SZ];
    uint32_t write_ptr;
    uint32_t read_ptr;
}iot_uart_tx_ring_t;

typedef struct __attribute__((packed, aligned(4))) {
    uint8_t  buf[UART_RX_BUF_SZ];
    uint32_t write_ptr;
    uint32_t read_ptr;
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
void _uart8_tx_dma_done(void);
void _uart8_tx_dma_touch(void);
void iot_uart8_tx_write(uint8_t *ptr, uint32_t len);

void iot_uart_init(void);

#endif /* I2C_HUB_UART_H */