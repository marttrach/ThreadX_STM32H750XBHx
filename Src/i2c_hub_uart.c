#include "i2c_hub_uart.h"
#include <string.h>
#include "main.h"
#include "iot.h"

static uint8_t  uart7_dma_rx_buf[UART_RX_BUF_SZ] __attribute__((aligned(32)));
static uint8_t  uart8_dma_rx_buf[UART_RX_BUF_SZ] __attribute__((aligned(32)));

static iot_uart_rx_ring_t uart7_rx_ring = {0};
static iot_uart_tx_ring_t uart7_tx_ring = {0};
static volatile uint16_t uart7_last_pos = 0;
static iot_uart_rx_ring_t uart8_rx_ring = {0};
static iot_uart_tx_ring_t uart8_tx_ring = {0};
static volatile uint16_t uart8_last_pos = 0;
#define UART7_TX (&uart7_tx_ring)
#define UART7_RX (&uart7_rx_ring)
#define UART8_TX (&uart8_tx_ring)
#define UART8_RX (&uart8_rx_ring)

static inline uint16_t dma_get_pos(const DMA_HandleTypeDef *hdma)
{
    return (uint16_t)(UART_RX_BUF_SZ - __HAL_DMA_GET_COUNTER(hdma));
}

static inline uint16_t dma_get_pos_stable(const DMA_HandleTypeDef *hdma)
{
    uint16_t p1, p2;
    do {
        p1 = dma_get_pos(hdma);
        __DSB();
        p2 = dma_get_pos(hdma);
    } while (p1 != p2);
    return p1;
}

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
        uint16_t cur_pos = dma_get_pos_stable(huart->hdmarx);
        if (cur_pos == uart7_last_pos) return;

        uint16_t delta = (cur_pos > uart7_last_pos)
                    ? (cur_pos - uart7_last_pos)
                    : (UART_RX_BUF_SZ - uart7_last_pos + cur_pos);

        iot_uart_rx_ring_t *rx = UART7_RX;
        uint32_t free = (rx->read_ptr > rx->write_ptr) ?
                        (rx->read_ptr - rx->write_ptr - 1U) :
                        (UART_RX_BUF_SZ - rx->write_ptr + rx->read_ptr - 1U);

        if (free == 0) {
            uart7_last_pos = cur_pos;
            return;
        }
        uint32_t to_copy = (delta > free) ? free : delta;
        uint32_t first_dma = UART_RX_BUF_SZ - uart7_last_pos;
        if (first_dma > to_copy) first_dma = to_copy;
        dcache_invalidate32_range(uart7_dma_rx_buf + uart7_last_pos, first_dma);
        uint32_t second_dma = to_copy - first_dma;
        if (second_dma) dcache_invalidate32_range(uart7_dma_rx_buf, second_dma);
        uint32_t wp = rx->write_ptr;
        uint32_t first_ring = UART_RX_BUF_SZ - wp;
        uint32_t first_copy = (first_ring > first_dma) ? first_dma : first_ring;
        memcpy(rx->buf + wp, uart7_dma_rx_buf + uart7_last_pos, first_copy);
        wp = (wp + first_copy) & (UART_RX_BUF_SZ - 1U);

        uint32_t remain_first = first_dma - first_copy;
        if (remain_first) {
            memcpy(rx->buf + wp, uart7_dma_rx_buf + uart7_last_pos + first_copy, remain_first);
            wp = (wp + remain_first) & (UART_RX_BUF_SZ - 1U);
        }
        if (second_dma) {
            uint32_t ring_left = UART_RX_BUF_SZ - wp;
            uint32_t second_copy1 = (ring_left > second_dma) ? second_dma : ring_left;
            memcpy(rx->buf + wp, uart7_dma_rx_buf, second_copy1);
            wp = (wp + second_copy1) & (UART_RX_BUF_SZ - 1U);

            uint32_t second_copy2 = second_dma - second_copy1;
            if (second_copy2) {
                memcpy(rx->buf + wp, uart7_dma_rx_buf + second_copy1, second_copy2);
                wp = (wp + second_copy2) & (UART_RX_BUF_SZ - 1U);
            }
        }
        rx->write_ptr  = wp;
        uart7_last_pos = cur_pos;
    }else if(huart == &huart8){
        uint16_t cur_pos = dma_get_pos_stable(huart->hdmarx);
        if (cur_pos == uart8_last_pos) return;

        uint16_t delta = (cur_pos > uart8_last_pos)
                    ? (cur_pos - uart8_last_pos)
                    : (UART_RX_BUF_SZ - uart8_last_pos + cur_pos);

        iot_uart_rx_ring_t *rx = UART8_RX;
        uint32_t free = (rx->read_ptr > rx->write_ptr) ?
                        (rx->read_ptr - rx->write_ptr - 1U) :
                        (UART_RX_BUF_SZ - rx->write_ptr + rx->read_ptr - 1U);

        if (free == 0) {
            uart8_last_pos = cur_pos;
            return;
        }

        uint32_t to_copy = (delta > free) ? free : delta;
        uint32_t first_dma = UART_RX_BUF_SZ - uart8_last_pos;
        if (first_dma > to_copy) first_dma = to_copy;
        dcache_invalidate32_range(uart8_dma_rx_buf + uart8_last_pos, first_dma);
        uint32_t second_dma = to_copy - first_dma;
        if (second_dma) dcache_invalidate32_range(uart8_dma_rx_buf, second_dma);
        uint32_t wp = rx->write_ptr;
        uint32_t first_ring = UART_RX_BUF_SZ - wp;
        uint32_t first_copy = (first_ring > first_dma) ? first_dma : first_ring;
        memcpy(rx->buf + wp, uart8_dma_rx_buf + uart8_last_pos, first_copy);
        wp = (wp + first_copy) & (UART_RX_BUF_SZ - 1U);
        uint32_t remain_first = first_dma - first_copy;
        if (remain_first) {
            memcpy(rx->buf + wp, uart8_dma_rx_buf + uart8_last_pos + first_copy, remain_first);
            wp = (wp + remain_first) & (UART_RX_BUF_SZ - 1U);
        }
        if (second_dma) {
            uint32_t ring_left = UART_RX_BUF_SZ - wp;
            uint32_t second_copy1 = (ring_left > second_dma) ? second_dma : ring_left;
            memcpy(rx->buf + wp, uart8_dma_rx_buf, second_copy1);
            wp = (wp + second_copy1) & (UART_RX_BUF_SZ - 1U);

            uint32_t second_copy2 = second_dma - second_copy1;
            if (second_copy2) {
                memcpy(rx->buf + wp, uart8_dma_rx_buf + second_copy1, second_copy2);
                wp = (wp + second_copy2) & (UART_RX_BUF_SZ - 1U);
            }
        }
        rx->write_ptr  = wp;
        uart8_last_pos = cur_pos;
    }
}

void uart7_rx_dma_start(void){
    HAL_UARTEx_ReceiveToIdle_DMA(&huart7, uart7_dma_rx_buf, UART_RX_BUF_SZ);
    // __HAL_DMA_DISABLE_IT(huart7.hdmarx, DMA_IT_HT | DMA_IT_TC);
    __HAL_DMA_ENABLE_IT(huart7.hdmarx, DMA_IT_HT | DMA_IT_TC);
    ((DMA_Stream_TypeDef *)huart7.hdmarx->Instance)->CR |= DMA_SxCR_CIRC;
    uart7_last_pos = dma_get_pos_stable(huart7.hdmarx);
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
    // __HAL_DMA_DISABLE_IT(huart8.hdmarx, DMA_IT_HT | DMA_IT_TC);
    __HAL_DMA_ENABLE_IT(huart8.hdmarx, DMA_IT_HT | DMA_IT_TC);
    ((DMA_Stream_TypeDef *)huart8.hdmarx->Instance)->CR |= DMA_SxCR_CIRC;
    uart8_last_pos = dma_get_pos_stable(huart8.hdmarx);
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
    if(huart == &huart7){
        _uart7_tx_dma_done();
        _uart7_tx_dma_touch();
    }else if(huart == &huart8){
        _uart8_tx_dma_done();
        _uart8_tx_dma_touch();
    }
}

void _uart7_tx_dma_done(void){
    UART7_TX->busy = 0;
}

void _uart7_tx_dma_touch(void){
    iot_uart_tx_ring_t *tx = UART7_TX;
    uint32_t w = tx->write_ptr;
    uint32_t r = tx->read_ptr;
    if (tx->busy) {
        HAL_UART_AbortTransmit(&huart7);
        DEBUG_DUMP(IOT_LOG_ERR, "uart7_tx_dma_touch: busy\r\n");
        return;
    }
    if (w == r) return;
    uint32_t data_len;
    uint8_t *src;
    if (w > r) {
        data_len = w - r;
        src = tx->buf + r;
    } else {
        data_len = UART_TX_BUF_SZ - r;
        src = tx->buf + r;
    }
    dcache_clean32_range(src, data_len);
    __DMB();
    HAL_StatusTypeDef st = HAL_UART_Transmit_DMA(&huart7, src, data_len);
    if (st == HAL_OK) {
        tx->busy = 1;
        tx->read_ptr = (w > r) ? w : 0;
    } else {
        DEBUG_DUMP(IOT_LOG_ERR, "uart7_tx_dma_touch error: HAL status=%d\r\n", st);
        tx->busy = 0;
    }
}

void iot_uart7_tx_write(uint8_t *ptr, uint32_t len){
    __disable_irq();
    iot_uart_tx_ring_t *tx = UART7_TX;
    uint32_t write_ptr = tx->write_ptr;
    uint32_t r = tx->read_ptr, w = tx->write_ptr;
    uint32_t free = (r > w) ? (r - w - 1U) : (UART_TX_BUF_SZ - w + r - 1U);
    if (len > free) len = free;
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
    uint32_t w = tx->write_ptr;
    uint32_t r = tx->read_ptr;
    if (tx->busy) {
        HAL_UART_AbortTransmit(&huart8);
        DEBUG_DUMP(IOT_LOG_ERR, "uart8_tx_dma_touch: busy\r\n");
        return;
    }
    if (w == r) return;
    uint32_t data_len;
    uint8_t *src;
    if (w > r) {
        data_len = w - r;
        src = tx->buf + r;
    } else {
        data_len = UART_TX_BUF_SZ - r;
        src = tx->buf + r;
    }
    dcache_clean32_range(src, data_len);
    __DMB();
    HAL_StatusTypeDef st = HAL_UART_Transmit_DMA(&huart8, src, data_len);
    if (st == HAL_OK) {
        tx->busy = 1;
        tx->read_ptr = (w > r) ? w : 0;
    } else {
        DEBUG_DUMP(IOT_LOG_ERR, "uart8_tx_dma_touch error: HAL status=%d\r\n", st);
        tx->busy = 0;
    }
}

void iot_uart8_tx_write(uint8_t *ptr, uint32_t len){
    __disable_irq();
    iot_uart_tx_ring_t *tx = UART8_TX;
    uint32_t write_ptr = tx->write_ptr;
    uint32_t r = tx->read_ptr, w = tx->write_ptr;
    uint32_t free = (r > w) ? (r - w - 1U) : (UART_TX_BUF_SZ - w + r - 1U);
    if (len > free) len = free;
    for(uint32_t i = 0; i < len ; i++ ){
        tx->buf[write_ptr] = ptr[i];
        write_ptr = (write_ptr + 1) & (UART_TX_BUF_SZ -1);
    }
    tx->write_ptr = write_ptr;
    _uart8_tx_dma_touch();
    __enable_irq();
}

/* Error Saver */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE)) {
        __HAL_UART_CLEAR_OREFLAG(huart);
    }
}

uint8_t *uart8_hub_helper(uint8_t *dst_ptr, hub_cmd_t *cmd, uint16_t avail){
    uint16_t need = MIN(cmd->len, avail);
    uint16_t frame_len = (need == 0) ? RSP_HDR_SZ + 4 : RSP_HDR_SZ + need + 4;
    dst_ptr = (uint8_t *)hub_sdram_alloc_tx(ALIGN32(frame_len));
    hub_rsp_t *rsp = (hub_rsp_t *)dst_ptr;
    rsp->status   = HUB_RSP_OK;
    rsp->len      = (need == 0) ? avail : need;
    rsp->reserved = 0;
    rsp->data_addr = cmd->data_addr;
    uint8_t *payload_dst = dst_ptr + RSP_HDR_SZ;
    uint32_t got = 0;
    while (got < need && avail != 0) {
        got += uart8_rx_read(payload_dst + got, need - got);
        if (!got) tx_thread_sleep(1);
    }
    uint32_t crc = iot_hub_crc32_hard(dst_ptr, frame_len - 4);
    memcpy(dst_ptr + frame_len - 4, &crc, 4);
    dcache_clean32_range(dst_ptr, frame_len);
    return dst_ptr; 
}

uint8_t *uart7_hub_helper(uint8_t *dst_ptr, hub_cmd_t *cmd, uint16_t avail){
    uint16_t need = MIN(cmd->len, avail);
    uint16_t frame_len = (need == 0) ? RSP_HDR_SZ + 4 : RSP_HDR_SZ + need + 4;
    dst_ptr = (uint8_t *)hub_sdram_alloc_tx(ALIGN32(frame_len));
    hub_rsp_t *rsp = (hub_rsp_t *)dst_ptr;
    rsp->status   = HUB_RSP_OK;
    rsp->len      = (need == 0) ? avail : need;
    rsp->reserved = 0;
    rsp->data_addr = cmd->data_addr;
    uint8_t *payload_dst = dst_ptr + RSP_HDR_SZ;
    uint32_t got = 0;
    while (got < need && avail != 0) {
        got += uart7_rx_read(payload_dst + got, need - got);
        if (!got) tx_thread_sleep(1);
    }
    uint32_t crc = iot_hub_crc32_hard(dst_ptr, frame_len - 4);
    memcpy(dst_ptr + frame_len - 4, &crc, 4);
    dcache_clean32_range(dst_ptr, frame_len);
    return dst_ptr; 
}