#include "usart.h"
#include "i2c_hub_uart.h"
#include <string.h>
#include "main.h"


extern UART_HandleTypeDef huart7;


static uint8_t  uart7_dma_rx_buf[UART_RX_BUF_SZ] __attribute__((aligned(4)));
static uint8_t  uart8_dma_rx_buf[UART_RX_BUF_SZ] __attribute__((aligned(4)));

static iot_uart_rx_ring_t uart7_rx_ring = {0};
static iot_uart_tx_ring_t uart7_tx_ring = {0};

static iot_uart_rx_ring_t uart8_rx_ring = {0};
static iot_uart_tx_ring_t uart8_tx_ring = {0};

#define UART7_TX (&uart7_tx_ring)
#define UART7_RX (&uart7_rx_ring)
#define UART8_TX (&uart8_tx_ring)
#define UART8_RX (&uart8_rx_ring)

void iot_uart_init(void)
{
    memset(&uart7_rx_ring, 0, sizeof(uart7_rx_ring));
    memset(&uart8_rx_ring, 0, sizeof(uart8_rx_ring));
    memset(&uart7_tx_ring, 0, sizeof(uart7_tx_ring));
    memset(&uart8_tx_ring, 0, sizeof(uart8_tx_ring));
    uart7_rx_dma_start();
    uart8_rx_dma_start();
}

/* RX */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
    if (huart == &huart7){
        iot_uart_rx_ring_t *rx = UART7_RX;
        uint32_t wp      = rx->write_ptr;
        uint32_t first   = UART_RX_BUF_SZ - wp;
        if (first > Size) first = Size;
        memcpy(rx->buf + wp, uart7_dma_rx_buf, first);
        if (Size > first) {
            memcpy(rx->buf, uart7_dma_rx_buf + first, Size - first);
        }
        rx->write_ptr = (wp + Size) & (UART_RX_BUF_SZ - 1U);
    }else if(huart == &huart8){
        iot_uart_rx_ring_t *rx = UART8_RX;
        uint32_t wp      = rx->write_ptr;
        uint32_t first   = UART_RX_BUF_SZ - wp;
        if (first > Size) first = Size;
        memcpy(rx->buf + wp, uart8_dma_rx_buf, first);
        if (Size > first) {
            memcpy(rx->buf, uart8_dma_rx_buf + first, Size - first);
        }
        rx->write_ptr = (wp + Size) & (UART_RX_BUF_SZ - 1U);
    }
}

void uart7_rx_dma_start(void){
    HAL_UARTEx_ReceiveToIdle_DMA(&huart7, uart7_dma_rx_buf, UART_RX_BUF_SZ);
    __HAL_DMA_DISABLE_IT(huart7.hdmarx, DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(huart7.hdmarx, DMA_IT_TC);
}

uint32_t uart7_rx_available(void){
    iot_uart_rx_ring_t *rx = UART7_RX;
    uint32_t w = rx->write_ptr;
    uint32_t r = rx->read_ptr;
    return (w >= r) ? (w - r) : (UART_RX_BUF_SZ - r + w);
}

uint32_t uart7_rx_read(uint8_t *dst, uint32_t len){
    iot_uart_rx_ring_t *rx = UART7_RX;
    uint32_t avail = uart7_rx_available();
    if (len > avail) len = avail;
    uint32_t r = rx->read_ptr;
    uint32_t first = UART_RX_BUF_SZ - r;
    if (first > len) first = len;
    memcpy(dst, rx->buf + r, first);
    if (len > first) {
        memcpy(dst + first, rx->buf, len - first);
    }
    rx->read_ptr = (r + len) & (UART_RX_BUF_SZ - 1U);
    return len;
}

void uart8_rx_dma_start(void){
    HAL_UARTEx_ReceiveToIdle_DMA(&huart8, uart8_dma_rx_buf, UART_RX_BUF_SZ);
    __HAL_DMA_DISABLE_IT(huart8.hdmarx, DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(huart8.hdmarx, DMA_IT_TC);
}

uint32_t uart8_rx_available(void){
    iot_uart_rx_ring_t *rx = UART8_RX;
    uint32_t w = rx->write_ptr;
    uint32_t r = rx->read_ptr;
    return (w >= r) ? (w - r) : (UART_RX_BUF_SZ - r + w);
}

uint32_t uart8_rx_read(uint8_t *dst, uint32_t len){
    iot_uart_rx_ring_t *rx = UART8_RX;
    uint32_t avail = uart8_rx_available();
    if (len > avail) len = avail;
    uint32_t r = rx->read_ptr;
    uint32_t first = UART_RX_BUF_SZ - r;
    if (first > len) first = len;
    memcpy(dst, rx->buf + r, first);
    if (len > first) {
        memcpy(dst + first, rx->buf, len - first);
    }
    rx->read_ptr = (r + len) & (UART_RX_BUF_SZ - 1U);
    return len;
}

/* TX */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    __disable_irq();
    if(huart == &huart7){
        _uart7_tx_dma_done();
        _uart7_tx_dma_touch();
    }else if(huart == &huart8){
        _uart8_tx_dma_done();
        _uart8_tx_dma_touch();
    }
    __enable_irq();
}

void _uart7_tx_dma_done(void){
    UART7_TX->busy = 0;
}

void _uart7_tx_dma_touch(void){
    iot_uart_tx_ring_t *tx = UART7_TX;
    uint32_t write_ptr = tx->write_ptr;
    uint32_t read_ptr = tx->read_ptr;

    if(tx->busy){
        return;
    }
    if(write_ptr != read_ptr){
        uint32_t data_len;
        tx->busy = 1;
        if(write_ptr > read_ptr){
            data_len = write_ptr - read_ptr;
            tx->read_ptr = write_ptr;
        } else {
            data_len = UART_TX_BUF_SZ - read_ptr;
            tx->read_ptr = 0;
        }
        HAL_UART_Transmit_DMA(&huart7, tx->buf+read_ptr, data_len);
    }
}

void iot_uart7_tx_write(uint8_t *ptr, uint32_t len){
    __disable_irq();
    iot_uart_tx_ring_t *tx = UART7_TX;
    uint32_t write_ptr = tx->write_ptr;
    for(uint32_t i = 0; i < len ; i++ ){
        tx->buf[write_ptr] = ptr[i];
        write_ptr = (write_ptr + 1) & (UART_TX_BUF_SZ -1);
    }
    tx->write_ptr = write_ptr;
    _uart7_tx_dma_touch();
    __enable_irq();
}

void _uart8_tx_dma_done(void){
    UART8_TX->busy = 0;
}

void _uart8_tx_dma_touch(void){
    iot_uart_tx_ring_t *tx = UART8_TX;
    uint32_t write_ptr = tx->write_ptr;
    uint32_t read_ptr = tx->read_ptr;

    if(tx->busy){
        return;
    }
    if(write_ptr != read_ptr){
        uint32_t data_len;
        tx->busy = 1;
        if(write_ptr > read_ptr){
            data_len = write_ptr - read_ptr;
            tx->read_ptr = write_ptr;
        } else {
            data_len = UART_TX_BUF_SZ - read_ptr;
            tx->read_ptr = 0;
        }
        HAL_UART_Transmit_DMA(&huart8, tx->buf+read_ptr, data_len);
    }
}

void iot_uart8_tx_write(uint8_t *ptr, uint32_t len){
    __disable_irq();
    iot_uart_tx_ring_t *tx = UART8_TX;
    uint32_t write_ptr = tx->write_ptr;
    for(uint32_t i = 0; i < len ; i++ ){
        tx->buf[write_ptr] = ptr[i];
        write_ptr = (write_ptr + 1) & (UART_TX_BUF_SZ -1);
    }
    tx->write_ptr = write_ptr;
    _uart8_tx_dma_touch();
    __enable_irq();
}