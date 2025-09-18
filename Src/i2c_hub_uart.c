#include "i2c_hub_uart.h"
#include <string.h>
#include "main.h"
#include "iot.h"
#include "stm32h7xx_hal_uart.h"

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

static TX_QUEUE  s_uart7_q;
static TX_THREAD s_uart7_thread;
static UCHAR     s_uart7_q_buf[32 * sizeof(ULONG)];
static UCHAR     s_uart7_stack[2048];
static volatile UINT s_uart7_started = 0;
static TX_SEMAPHORE s_uart7_tx_gate;
static TX_SEMAPHORE s_uart7_tx_done;

static TX_QUEUE  s_uart8_q;
static TX_THREAD s_uart8_thread;
static UCHAR     s_uart8_q_buf[32 * sizeof(ULONG)];
static UCHAR     s_uart8_stack[2048];
static volatile UINT s_uart8_started = 0;
static TX_SEMAPHORE s_uart8_tx_gate;
static TX_SEMAPHORE s_uart8_tx_done;

#ifndef UART_RECOVER_ESCALATION_MS
#define UART_RECOVER_ESCALATION_MS   50U
#endif
#ifndef UART_RECOVER_THRESHOLD
#define UART_RECOVER_THRESHOLD       3U
#endif
#ifndef UART_TX_BOUNCE_CHUNK
#define UART_TX_BOUNCE_CHUNK  1024U
#endif
#ifndef TX_DMA_TC_FLAG
#define TX_DMA_TC_FLAG   DMA_FLAG_TCIF0_4
#define TX_DMA_ERR_FLAGS (DMA_FLAG_TEIF0_4 | DMA_FLAG_DMEIF0_4 | DMA_FLAG_FEIF0_4)
#endif
#define UART_RESP_SLOTS  4 

static volatile uint32_t s_u7_err_tick = 0, s_u8_err_tick = 0;
static volatile uint8_t  s_u7_err_cnt  = 0, s_u8_err_cnt  = 0;
static volatile UINT s_uart_tx_sem_inited = 0U;
static uint8_t s_uart_bounce[UART_TX_BOUNCE_CHUNK] __attribute__((aligned(32)));  /* AXI DMA need 32B align */

typedef struct {
    uint8_t     *frame;
    hub_tx_task_t task;
    volatile uint8_t busy;
} uart_resp_slot_t;

typedef struct {
    uart_resp_slot_t slot[UART_RESP_SLOTS];
    uint32_t         cap;
    uint8_t          wr;
} uart_resp_ctx_t;

typedef struct {
    UART_HandleTypeDef *huart;
    TX_SEMAPHORE       *gate;
    TX_SEMAPHORE       *done;
} uart_tx_sem_map_t;

static uart_resp_ctx_t s_u7_ctx = {0};
static uart_resp_ctx_t s_u8_ctx = {0};

static VOID uart7_thread_entry(ULONG arg);
static VOID uart8_thread_entry(ULONG arg);

/* Auto Map U7/8 START */
static inline void _uart_port_map(UART_HandleTypeDef *huart, uint8_t **rx_dma_buf, volatile uint16_t **plast_pos, iot_uart_tx_ring_t **ptx_ring)
{
    if (huart == &huart7) {
        *rx_dma_buf = uart7_dma_rx_buf;
        *plast_pos  = &uart7_last_pos;
        *ptx_ring   = UART7_TX;
    } else {
        *rx_dma_buf = uart8_dma_rx_buf;
        *plast_pos  = &uart8_last_pos;
        *ptx_ring   = UART8_TX;
    }
}

static inline uart_tx_sem_map_t uart_tx_map(UART_HandleTypeDef *huart)
{
    uart_tx_sem_map_t m = {0};
    if (huart == &huart7) {
        m.huart = &huart7; m.gate = &s_uart7_tx_gate; m.done = &s_uart7_tx_done;
    } else if (huart == &huart8) {
        m.huart = &huart8; m.gate = &s_uart8_tx_gate; m.done = &s_uart8_tx_done;
    }
    return m;
}
/* Auto Map U7/8 END */
/* Utility Functions START*/
static uart_resp_slot_t *uart_resp_take_free(uart_resp_ctx_t *ctx)
{
    for (int i = 0; i < UART_RESP_SLOTS; ++i) {
        uint8_t idx = (ctx->wr + i) & (UART_RESP_SLOTS - 1);
        if (!ctx->slot[idx].busy) {
            ctx->wr = (idx + 1) & (UART_RESP_SLOTS - 1);
            ctx->slot[idx].busy = 1;
            return &ctx->slot[idx];
        }
    }
    return NULL;
}

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

static void uart_rx_drain_dma(UART_HandleTypeDef *huart,
                              iot_uart_rx_ring_t *rx,
                              uint8_t *dma_buf,
                              volatile uint16_t *plast_pos)
{
    uint16_t cur_pos = dma_get_pos_stable(huart->hdmarx);
    if (cur_pos == *plast_pos) return;

    uint16_t delta = (cur_pos > *plast_pos)
                   ? (cur_pos - *plast_pos)
                   : (UART_RX_BUF_SZ - *plast_pos + cur_pos);

    uint32_t free = (rx->read_ptr > rx->write_ptr)
                  ? (rx->read_ptr - rx->write_ptr - 1U)
                  : (UART_RX_BUF_SZ - rx->write_ptr + rx->read_ptr - 1U);

    if (free == 0) {
        *plast_pos = cur_pos;
        return;
    }

    uint32_t to_copy = (delta > free) ? free : delta;

    uint32_t first_dma = UART_RX_BUF_SZ - *plast_pos;
    if (first_dma > to_copy) first_dma = to_copy;

    dcache_invalidate32_range(dma_buf + *plast_pos, first_dma);

    uint32_t wp = rx->write_ptr;
    uint32_t first_ring = UART_RX_BUF_SZ - wp;
    uint32_t first_copy = (first_ring > first_dma) ? first_dma : first_ring;
    memcpy(rx->buf + wp, dma_buf + *plast_pos, first_copy);
    wp = (wp + first_copy) & (UART_RX_BUF_SZ - 1U);

    uint32_t remain_first = first_dma - first_copy;
    if (remain_first) {
        memcpy(rx->buf + wp, dma_buf + *plast_pos + first_copy, remain_first);
        wp = (wp + remain_first) & (UART_RX_BUF_SZ - 1U);
    }

    uint32_t second_dma = to_copy - first_dma;
    if (second_dma) {
        dcache_invalidate32_range(dma_buf, second_dma);

        uint32_t ring_left = UART_RX_BUF_SZ - wp;
        uint32_t second_copy1 = (ring_left > second_dma) ? second_dma : ring_left;
        memcpy(rx->buf + wp, dma_buf, second_copy1);
        wp = (wp + second_copy1) & (UART_RX_BUF_SZ - 1U);

        uint32_t second_copy2 = second_dma - second_copy1;
        if (second_copy2) {
            memcpy(rx->buf + wp, dma_buf + second_copy1, second_copy2);
            wp = (wp + second_copy2) & (UART_RX_BUF_SZ - 1U);
        }
    }

    rx->write_ptr  = wp;
    *plast_pos     = cur_pos;
    __DMB();
}

static void uart_ctx_done_cb(hub_tx_task_t *t)
{
    if (!t || !t->user_ctx) return;
    uart_resp_slot_t *slot = (uart_resp_slot_t *)t->user_ctx;
    slot->busy = 0;
}
/* Utility Functions END*/

/* Init START*/
static void uart_tx_sem_init_once(void)
{
    if (s_uart_tx_sem_inited) return;

    tx_semaphore_create(&s_uart7_tx_gate, "uart7_tx_gate", 1);
    tx_semaphore_create(&s_uart7_tx_done, "uart7_tx_done", 0);

    tx_semaphore_create(&s_uart8_tx_gate, "uart8_tx_gate", 1);
    tx_semaphore_create(&s_uart8_tx_done, "uart8_tx_done", 0);

    s_uart_tx_sem_inited = 1U;
}

static int uart_resp_ctx_init(uart_resp_ctx_t *ctx)
{
    ctx->cap = (uint32_t)(RSP_HDR_SZ + UART_RX_BUF_SZ + 4U);
    for (int i = 0; i < UART_RESP_SLOTS; ++i) {
        ctx->slot[i].frame = (uint8_t*)hub_heap_alloc_aligned(ALIGN32(ctx->cap), HUB_DMA_ALIGN);
        if (!ctx->slot[i].frame) return 0;
        ctx->slot[i].busy  = 0;
        ctx->slot[i].task.buf         = ctx->slot[i].frame;
        ctx->slot[i].task.total       = 0;
        ctx->slot[i].task.sent        = 0;
        ctx->slot[i].task.stage_tx    = TX_STAGE_HDR;
        ctx->slot[i].task.alloc_flags = HUB_TASK_F_STATIC_BUF | HUB_TASK_F_STATIC_TASK;
        ctx->slot[i].task.done_cb     = uart_ctx_done_cb;
        ctx->slot[i].task.user_ctx    = &ctx->slot[i];
    }
    ctx->wr = 0;
    return 1;
}

void iot_uart_init(void)
{
    memset(&uart7_rx_ring, 0, sizeof(uart7_rx_ring));
    memset(&uart8_rx_ring, 0, sizeof(uart8_rx_ring));
    memset(&uart7_tx_ring, 0, sizeof(uart7_tx_ring));
    memset(&uart8_tx_ring, 0, sizeof(uart8_tx_ring));
    uart7_rx_dma_start();
    uart8_rx_dma_start();
    uart_tx_sem_init_once();
}
/* Init END*/

/* RX START */
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart){ (void)huart; }
void HAL_UART_RxCpltCallback    (UART_HandleTypeDef *huart){ (void)huart; }

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

static inline void uart7_rx_sync_from_dma(void){
    uart_rx_drain_dma(&huart7, UART7_RX, uart7_dma_rx_buf, &uart7_last_pos);
}

void uart7_rx_dma_start(void){
    HAL_UART_Receive_DMA(&huart7, uart7_dma_rx_buf, UART_RX_BUF_SZ);
    __HAL_DMA_ENABLE_IT(huart7.hdmarx, DMA_IT_HT | DMA_IT_TC);
    ((DMA_Stream_TypeDef *)huart7.hdmarx->Instance)->CR |= DMA_SxCR_CIRC;
    uart7_last_pos = dma_get_pos_stable(huart7.hdmarx);
}

uint32_t uart7_rx_available(void){
    uart7_rx_sync_from_dma();
    iot_uart_rx_ring_t *rx = UART7_RX;
    uint32_t w = rx->write_ptr, r = rx->read_ptr;
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

static inline void uart8_rx_sync_from_dma(void){
    uart_rx_drain_dma(&huart8, UART8_RX, uart8_dma_rx_buf, &uart8_last_pos);
}

void uart8_rx_dma_start(void){
    HAL_UART_Receive_DMA(&huart8, uart8_dma_rx_buf, UART_RX_BUF_SZ);
    __HAL_DMA_ENABLE_IT(huart8.hdmarx, DMA_IT_HT | DMA_IT_TC);
    ((DMA_Stream_TypeDef *)huart8.hdmarx->Instance)->CR |= DMA_SxCR_CIRC;
    uart8_last_pos = dma_get_pos_stable(huart8.hdmarx);
}

uint32_t uart8_rx_available(void){
    uart8_rx_sync_from_dma();
    iot_uart_rx_ring_t *rx = UART8_RX;
    uint32_t w = rx->write_ptr, r = rx->read_ptr;
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
/* RX END */
/* TX START */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
#if IOT_UART_TX_USE_BLOCKING
    uart_tx_sem_map_t m = uart_tx_map(huart);
    if (m.done) {
        tx_semaphore_put(m.done);
    }
#else
    if(huart == &huart7){
        _uart7_tx_dma_done();
        _uart7_tx_dma_touch();
    }else if(huart == &huart8){
        _uart8_tx_dma_done();
        _uart8_tx_dma_touch();
    }
#endif
}

static inline void _uart_tx_dma_blocking(UART_HandleTypeDef *huart, const uint8_t *src, uint32_t len)
{
    if (src == NULL || len == 0U) return;

    uart_tx_sem_init_once();
    uart_tx_sem_map_t m = uart_tx_map(huart);
    if (m.huart == NULL || m.gate == NULL || m.done == NULL) return;

    if (tx_semaphore_get(m.gate, TX_WAIT_FOREVER) != TX_SUCCESS) return;

    HAL_StatusTypeDef st;

    if ((((uintptr_t)src & 0xF0000000u) == 0xC0000000u)) {
        uint32_t remain = len;
        const uint8_t *p = src;
        while (remain) {
            uint32_t chunk = (remain > UART_TX_BOUNCE_CHUNK) ? UART_TX_BOUNCE_CHUNK : remain;
            memcpy(s_uart_bounce, p, chunk);
            dcache_clean32_range(s_uart_bounce, chunk);

            st = HAL_UART_Transmit_DMA(huart, s_uart_bounce, (uint16_t)chunk);
            if (st != HAL_OK) {
                tx_semaphore_put(m.gate);
                return;
            }
            if (tx_semaphore_get(m.done, TX_WAIT_FOREVER) != TX_SUCCESS) {
                (void)HAL_UART_AbortTransmit(huart);
                tx_semaphore_put(m.gate);
                return;
            }
            p      += chunk;
            remain -= chunk;
        }
    } else {
        dcache_clean32_range(src, len);
        st = HAL_UART_Transmit_DMA(huart, (uint8_t *)src, (uint16_t)len);
        if (st != HAL_OK) {
            tx_semaphore_put(m.gate);
            return;
        }
        (void)tx_semaphore_get(m.done, TX_WAIT_FOREVER);
    }
    tx_semaphore_put(m.gate);
}

void _uart7_tx_dma_done(void){
    UART7_TX->busy = 0;
}

void _uart7_tx_dma_touch(void)
{
    static uint32_t s_guard_ts = 0;
    const  uint32_t GUARD_MS   = 5; 

    iot_uart_tx_ring_t *tx = UART7_TX;
    uint32_t w = tx->write_ptr, r = tx->read_ptr;

    if (tx->busy && huart7.hdmatx &&
        HAL_DMA_GetState(huart7.hdmatx) == HAL_DMA_STATE_READY) {
        tx->busy   = 0;
        s_guard_ts = 0;
    }

    if (tx->busy && huart7.hdmatx) {
        if (__HAL_DMA_GET_FLAG(huart7.hdmatx, TX_DMA_TC_FLAG)) {
            HAL_DMA_IRQHandler(huart7.hdmatx);
            s_guard_ts = 0;
            return;
        }
        if (__HAL_DMA_GET_FLAG(huart7.hdmatx, TX_DMA_ERR_FLAGS)) {
            __HAL_DMA_CLEAR_FLAG(huart7.hdmatx, TX_DMA_ERR_FLAGS);
            HAL_UART_IRQHandler(&huart7);
            s_guard_ts = 0;
            return;
        }
        if (HAL_DMA_GetState(huart7.hdmatx) == HAL_DMA_STATE_BUSY) {
            uint32_t now = HAL_GetTick();
            if (s_guard_ts == 0) s_guard_ts = now;
            else if ((now - s_guard_ts) > GUARD_MS) {
                (void)HAL_UART_AbortTransmit(&huart7);
                tx->busy   = 0;
                s_guard_ts = 0;
            }
            return;
        }
    }

    if (tx->busy) return;
    if (w == r)   { s_guard_ts = 0; return; }

    uint32_t data_len;
    uint8_t *src;
    if (w > r) { data_len = w - r;          src = tx->buf + r; }
    else       { data_len = UART_TX_BUF_SZ - r; src = tx->buf + r; }

    dcache_clean32_range(src, data_len);
    __DMB();

    if (huart7.hdmatx && HAL_DMA_GetState(huart7.hdmatx) != HAL_DMA_STATE_READY) return;

    HAL_StatusTypeDef st = HAL_UART_Transmit_DMA(&huart7, src, data_len);
    if (st == HAL_OK) {
        tx->busy     = 1;
        tx->read_ptr = (w > r) ? w : 0;
        s_guard_ts   = HAL_GetTick();
        return;
    }
    if (st == HAL_BUSY) {
        return;
    }

    if ((SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) == 0u) {
        uint32_t n = (data_len > 64U) ? 64U : data_len;
        if (HAL_UART_Transmit(&huart7, src, n, 100) == HAL_OK) {
            tx->read_ptr = (r + n) & (UART_TX_BUF_SZ - 1U);
        }
    }
}

void iot_uart7_tx_write(uint8_t *src, uint32_t len)
{
#if IOT_UART_TX_USE_BLOCKING
    _uart_tx_dma_blocking(&huart7, src, len);
#else
    iot_uart_tx_ring_t *tx = UART7_TX;

    __disable_irq();
    uint32_t r = tx->read_ptr;
    uint32_t w = tx->write_ptr;
    __enable_irq();

    uint32_t free = (r > w) ? (r - w - 1U) : (UART_TX_BUF_SZ - w + r - 1U);
    if (len > free) len = free;
    if (len == 0U) { _uart7_tx_dma_touch(); return; }

    uint32_t first = UART_TX_BUF_SZ - w;
    if (first > len) first = len;
    memcpy(tx->buf + w, src, first);
    if (len > first) {
        memcpy(tx->buf, src + first, len - first);
    }
    __DMB();

    __disable_irq();
    tx->write_ptr = (w + len) & (UART_TX_BUF_SZ - 1U);
    __enable_irq();

    _uart7_tx_dma_touch();
#endif
}

void _uart8_tx_dma_done(void){
    UART8_TX->busy = 0;
}

void _uart8_tx_dma_touch(void)
{
    static uint32_t s_guard_ts = 0;
    const  uint32_t GUARD_MS   = 5; 

    iot_uart_tx_ring_t *tx = UART8_TX;
    uint32_t w = tx->write_ptr, r = tx->read_ptr;

    if (tx->busy && huart8.hdmatx &&
        HAL_DMA_GetState(huart8.hdmatx) == HAL_DMA_STATE_READY) {
        tx->busy   = 0;
        s_guard_ts = 0;
    }

    if (tx->busy && huart8.hdmatx) {
        if (__HAL_DMA_GET_FLAG(huart8.hdmatx, TX_DMA_TC_FLAG)) {
            HAL_DMA_IRQHandler(huart8.hdmatx);
            s_guard_ts = 0;
            return;
        }
        if (__HAL_DMA_GET_FLAG(huart8.hdmatx, TX_DMA_ERR_FLAGS)) {
            __HAL_DMA_CLEAR_FLAG(huart8.hdmatx, TX_DMA_ERR_FLAGS);
            HAL_UART_IRQHandler(&huart8);
            s_guard_ts = 0;
            return;
        }
        if (HAL_DMA_GetState(huart8.hdmatx) == HAL_DMA_STATE_BUSY) {
            uint32_t now = HAL_GetTick();
            if (s_guard_ts == 0) s_guard_ts = now;
            else if ((now - s_guard_ts) > GUARD_MS) {
                (void)HAL_UART_AbortTransmit(&huart8);
                tx->busy   = 0;
                s_guard_ts = 0;
            }
            return;
        }
    }

    if (tx->busy) return;
    if (w == r)   { s_guard_ts = 0; return; }

    uint32_t data_len;
    uint8_t *src;
    if (w > r) { data_len = w - r;          src = tx->buf + r; }
    else       { data_len = UART_TX_BUF_SZ - r; src = tx->buf + r; }

    dcache_clean32_range(src, data_len);
    __DMB();

    if (huart8.hdmatx && HAL_DMA_GetState(huart8.hdmatx) != HAL_DMA_STATE_READY) return;

    HAL_StatusTypeDef st = HAL_UART_Transmit_DMA(&huart8, src, data_len);
    if (st == HAL_OK) {
        tx->busy     = 1;
        tx->read_ptr = (w > r) ? w : 0;
        s_guard_ts   = HAL_GetTick();
        return;
    }
    if (st == HAL_BUSY) {
        return;
    }

    if ((SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) == 0u) {
        uint32_t n = (data_len > 64U) ? 64U : data_len;
        if (HAL_UART_Transmit(&huart8, src, n, 100) == HAL_OK) {
            tx->read_ptr = (r + n) & (UART_TX_BUF_SZ - 1U);
        }
    }
}

void iot_uart8_tx_write(uint8_t *src, uint32_t len)
{
#if IOT_UART_TX_USE_BLOCKING
    _uart_tx_dma_blocking(&huart8, src, len);
#else
    iot_uart_tx_ring_t *tx = UART8_TX;

    __disable_irq();
    uint32_t r = tx->read_ptr;
    uint32_t w = tx->write_ptr;
    __enable_irq();

    uint32_t free = (r > w) ? (r - w - 1U) : (UART_TX_BUF_SZ - w + r - 1U);
    if (len > free) len = free;
    if (len == 0U) { _uart8_tx_dma_touch(); return; }

    uint32_t first = UART_TX_BUF_SZ - w;
    if (first > len) first = len;
    memcpy(tx->buf + w, src, first);
    if (len > first) {
        memcpy(tx->buf, src + first, len - first);
    }
    __DMB();

    __disable_irq();
    tx->write_ptr = (w + len) & (UART_TX_BUF_SZ - 1U);
    __enable_irq();

    _uart8_tx_dma_touch();
#endif
}
/* TX END */
/* Error Saver START*/
static inline void _uart_clear_all_errors_and_flush(UART_HandleTypeDef *huart)
{
    CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);
    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_IDLEFLAG(huart);

    __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);

    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE) != RESET) {
        (void)huart->Instance->RDR;
    }
    SET_BIT(huart->Instance->CR3, USART_CR3_EIE);
}

static int _uart_restart_dma(UART_HandleTypeDef *huart, uint8_t *rx_dma_buf, volatile uint16_t *plast_pos)
{
    CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAR | USART_CR3_DMAT);

    if (huart->hdmarx) {
        if (HAL_DMA_GetState(huart->hdmarx) != HAL_DMA_STATE_READY) {
            (void)HAL_DMA_Abort(huart->hdmarx);
        }
        __HAL_DMA_DISABLE(huart->hdmarx);
        __HAL_DMA_CLEAR_FLAG(huart->hdmarx,
            DMA_FLAG_TEIF0_4 | DMA_FLAG_DMEIF0_4 | DMA_FLAG_FEIF0_4 |
            DMA_FLAG_HTIF0_4 | DMA_FLAG_TCIF0_4);
    }
    if (huart->hdmatx) {
        if (HAL_DMA_GetState(huart->hdmatx) != HAL_DMA_STATE_READY) {
            (void)HAL_DMA_Abort(huart->hdmatx);
        }
        __HAL_DMA_DISABLE(huart->hdmatx);
        __HAL_DMA_CLEAR_FLAG(huart->hdmatx,
            DMA_FLAG_TEIF0_4 | DMA_FLAG_DMEIF0_4 | DMA_FLAG_FEIF0_4 |
            DMA_FLAG_HTIF0_4 | DMA_FLAG_TCIF0_4);
    }

    DEBUG_DUMP(IOT_LOG_DEBUG, "UART recover: clear flags + flush (inst=%p)\r\n", huart->Instance);
    CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);
    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE) != RESET) {
        (void)huart->Instance->RDR;
    }
    SET_BIT(huart->Instance->CR3, USART_CR3_EIE);

    huart->ErrorCode = HAL_UART_ERROR_NONE;
    huart->gState    = HAL_UART_STATE_READY;
    huart->RxState   = HAL_UART_STATE_READY;

    dcache_invalidate32_range(rx_dma_buf, UART_RX_BUF_SZ);
    if (HAL_UART_Receive_DMA(huart, rx_dma_buf, UART_RX_BUF_SZ) != HAL_OK) {
        DEBUG_DUMP(IOT_LOG_DEBUG, "UART recover: HAL_UART_Receive_DMA failed (inst=%p)\r\n", huart->Instance);
        return 0;
    }
    if (huart->hdmarx) {
        __HAL_DMA_ENABLE_IT(huart->hdmarx, DMA_IT_HT | DMA_IT_TC);
        ((DMA_Stream_TypeDef *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CIRC;
    }
    *plast_pos = dma_get_pos_stable(huart->hdmarx);

    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

    DEBUG_DUMP(IOT_LOG_DEBUG, "UART recover: RX/TX DMA sanitized + RX restarted (inst=%p)\r\n", huart->Instance);
    return 1;
}

static void uart_busy_kill(UART_HandleTypeDef *huart, iot_uart_tx_ring_t *tx, uint32_t err)
{
    if (!tx || !tx->busy) return;
    DEBUG_DUMP(IOT_LOG_DEBUG, "UART recover: TX busy → abort TX DMA (inst=%p, err=0x%08lX)\r\n",
               huart->Instance, (unsigned long)err);
    (void)HAL_UART_AbortTransmit(huart);
    tx->busy = 0;
}

static void uart_err_recover(UART_HandleTypeDef *huart, const char *why)
{
    uint32_t err = HAL_UART_GetError(huart);
    uint8_t *rx_dma_buf = NULL;
    volatile uint16_t *plast_pos = NULL;
    iot_uart_tx_ring_t *tx = NULL;

    DEBUG_DUMP(IOT_LOG_DEBUG, "UART recover called (inst=%p err=0x%08lX)\r\n",
               huart->Instance, (unsigned long)err);

    _uart_port_map(huart, &rx_dma_buf, &plast_pos, &tx);
    DEBUG_DUMP(IOT_LOG_DEBUG, "UART recover: mapped (rx_dma_buf=%p plast_pos=%p tx=%p)\r\n",
               rx_dma_buf, plast_pos, tx);

    (void)_uart_restart_dma(huart, rx_dma_buf, plast_pos);

    uint32_t now = HAL_GetTick();
    volatile uint32_t *ptick = (huart == &huart7) ? &s_u7_err_tick : &s_u8_err_tick;
    volatile uint8_t  *pcnt  = (huart == &huart7) ? &s_u7_err_cnt  : &s_u8_err_cnt;

    if ((now - *ptick) <= UART_RECOVER_ESCALATION_MS) {
        if (*pcnt < 0xFF) (*pcnt)++;
    } else {
        *pcnt = 1;
    }
    *ptick = now;

    DEBUG_DUMP(IOT_LOG_DEBUG, "UART recover(%s): inst=%p err=0x%08lX cnt=%u\r\n",
               why, huart->Instance, (unsigned long)err, *pcnt);

    uart_busy_kill(huart, tx, err);

    if (*pcnt >= UART_RECOVER_THRESHOLD) {
        DEBUG_DUMP(IOT_LOG_DEBUG, "UART recover: escalation (deinit/init)\r\n");
        (void)HAL_UART_DeInit(huart);
        (void)HAL_UART_Init(huart);
        (void)_uart_restart_dma(huart, rx_dma_buf, plast_pos);
        *pcnt = 0;
    }
    if (tx) { tx->busy = 0; }
    if (huart == &huart7) _uart7_tx_dma_touch();
    else if (huart == &huart8) _uart8_tx_dma_touch();

    DEBUG_DUMP(IOT_LOG_DEBUG, "UART recover: done (inst=%p)\r\n", huart->Instance);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->hdmarx) {
        uint32_t rxerr = huart->hdmarx->ErrorCode;
        if (rxerr) DEBUG_DUMP(IOT_LOG_ERR, "UART DMA RX err=0x%08lX\r\n", (unsigned long)rxerr);
    }
    if (huart->hdmatx) {
        uint32_t txerr = huart->hdmatx->ErrorCode;
        if (txerr) DEBUG_DUMP(IOT_LOG_ERR, "UART DMA TX err=0x%08lX\r\n", (unsigned long)txerr);
    }
    uart_err_recover(huart, "HAL_UART_ErrorCallback");
    uart_tx_sem_map_t m = uart_tx_map(huart);
    if (m.done) {
        tx_semaphore_put(m.done);
    }
}

void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *huart)
{
    uart_tx_sem_map_t m = uart_tx_map(huart);
    if (m.done) {
        tx_semaphore_put(m.done);
    }
}
/* Error Saver END*/
/* Thread START */
void uart7_thread_start(void)
{
    if (s_uart7_started) return;
    if (!uart_resp_ctx_init(&s_u7_ctx)) return;
    tx_queue_create(&s_uart7_q, "uart7_q", TX_1_ULONG, s_uart7_q_buf, sizeof(s_uart7_q_buf));
    tx_thread_create(&s_uart7_thread, "uart7_thread", uart7_thread_entry, 0,
                     s_uart7_stack, sizeof(s_uart7_stack),
                     12, 12, TX_NO_TIME_SLICE, TX_AUTO_START);
    s_uart7_started = 1;
}

int uart7_post_read(uint16_t len, uint32_t data_addr)
{
    if (!s_uart7_started) return 0;
    uart_cmd_msg_t msg = { .type = UART_CMD_READ, .len = len, .data_addr = data_addr };
    return (tx_queue_send(&s_uart7_q, &msg, TX_NO_WAIT) == TX_SUCCESS);
}

static VOID uart7_thread_entry(ULONG arg)
{
    (void)arg;
    for (;;) {
        uart_cmd_msg_t msg;
        if (tx_queue_receive(&s_uart7_q, &msg, TX_WAIT_FOREVER) != TX_SUCCESS) continue;
        if (msg.type == UART_CMD_READ) {
            uart_resp_slot_t *slot = uart_resp_take_free(&s_u7_ctx);
            if (!slot) { continue; }
            uint8_t   *dst = slot->frame;
            hub_rsp_t *rsp = (hub_rsp_t*)dst;
            uint16_t req = msg.len;
            uint16_t avail = (uint16_t)uart7_rx_available();
            uint16_t need = (req == 0U) ? 0U : ((req <= avail) ? req : avail);
            rsp->status    = HUB_RSP_OK;
            rsp->reserved  = 0;
            rsp->data_addr = msg.data_addr;
            uint32_t frame_len = 0U;
            uint32_t got       = 0U;
            if (req == 0U) {
                rsp->len   = avail;
                frame_len  = RSP_HDR_SZ + 4U;
            } else {
                uint8_t *payload = dst + RSP_HDR_SZ;
                while (got < need) {
                    uint32_t g = uart7_rx_read(payload + got, (uint32_t)(need - got));
                    if (g == 0U) break;
                    got += g;
                }
                rsp->len  = (uint16_t)got;
                frame_len = RSP_HDR_SZ + (uint32_t)got + 4U;
            }

            uint32_t crc = iot_hub_crc32_hard(dst, frame_len - 4U);
            memcpy(dst + frame_len - 4U, &crc, 4U);

            dcache_clean32_range(dst, frame_len);
            __DMB();
            if (rsp->len > 0U) {
                dcache_clean32_range(dst + RSP_HDR_SZ, rsp->len);   
                DEBUG_DUMP(IOT_LOG_ALL,
                    "UART7 READ req=%u avail=%u need=%u got=%u frm=%lu len=%u\r\n",
                    (unsigned)req, (unsigned)avail, (unsigned)need, (unsigned)got,
                    (unsigned long)frame_len, (unsigned)rsp->len);
            }
            if (req != 0U && got > 0U) {
                DEBUG_DUMP(IOT_LOG_ALL, "UART7 rsp bytes: ");
                for (uint32_t i = 0; i < (unsigned)( rsp->len ); i++) {
                    DEBUG_DUMP(IOT_LOG_ALL, "0x%02X ", dst[RSP_HDR_SZ + i]);
                }
                DEBUG_DUMP(IOT_LOG_ALL, "\r\n");
            }
            slot->task.total = frame_len; 
            (void)hub_send_tx_task(&slot->task);
        }
    }
}

void uart8_thread_start(void)
{
    if (s_uart8_started) return;
    if (!uart_resp_ctx_init(&s_u8_ctx)) return;
    tx_queue_create(&s_uart8_q, "uart8_q", TX_1_ULONG, s_uart8_q_buf, sizeof(s_uart8_q_buf));
    tx_thread_create(&s_uart8_thread, "uart8_thread", uart8_thread_entry, 0,
                     s_uart8_stack, sizeof(s_uart8_stack),
                     12, 12, TX_NO_TIME_SLICE, TX_AUTO_START);
    s_uart8_started = 1;
}

int uart8_post_read(uint16_t len, uint32_t data_addr)
{
    if (!s_uart8_started) return 0;
    uart_cmd_msg_t msg = { .type = UART_CMD_READ, .len = len, .data_addr = data_addr };
    return (tx_queue_send(&s_uart8_q, &msg, TX_NO_WAIT) == TX_SUCCESS);
}

static VOID uart8_thread_entry(ULONG arg)
{
    (void)arg;
    for (;;) {
        uart_cmd_msg_t msg;
        if (tx_queue_receive(&s_uart8_q, &msg, TX_WAIT_FOREVER) != TX_SUCCESS) continue;
        if (msg.type == UART_CMD_READ) {
            uart_resp_slot_t *slot = uart_resp_take_free(&s_u8_ctx);
            if (!slot) { continue; }

            uint8_t   *dst = slot->frame;
            hub_rsp_t *rsp = (hub_rsp_t*)dst;
            uint16_t req = msg.len;
            uint16_t avail = (uint16_t)uart8_rx_available();
            uint16_t need = (req == 0U) ? 0U : ((req <= avail) ? req : avail);
            rsp->status    = HUB_RSP_OK;
            rsp->reserved  = 0;
            rsp->data_addr = msg.data_addr;
            uint32_t frame_len = 0U;
            uint32_t got       = 0U;
            if (req == 0U) {
                rsp->len   = avail;
                frame_len  = RSP_HDR_SZ + 4U;
            } else {
                uint8_t *payload = dst + RSP_HDR_SZ;
                while (got < need) {
                    uint32_t g = uart8_rx_read(payload + got, (uint32_t)(need - got));
                    if (g == 0U) break;
                    got += g;
                }
                rsp->len  = (uint16_t)got;
                frame_len = RSP_HDR_SZ + (uint32_t)got + 4U;
            }

            uint32_t crc = iot_hub_crc32_hard(dst, frame_len - 4U);
            memcpy(dst + frame_len - 4U, &crc, 4U);
            dcache_clean32_range(dst, frame_len);
            __DMB();
            if (rsp->len > 0U) {
                dcache_clean32_range(dst + RSP_HDR_SZ, rsp->len);   
                DEBUG_DUMP(IOT_LOG_ALL,
                    "UART8 READ req=%u avail=%u need=%u got=%u frm=%lu len=%u\r\n",
                    (unsigned)req, (unsigned)avail, (unsigned)need, (unsigned)got,
                    (unsigned long)frame_len, (unsigned)rsp->len);
            }
            if (req != 0U && got > 0U) {
                DEBUG_DUMP(IOT_LOG_ALL, "UART8 rsp bytes: ");
                for (uint32_t i = 0; i < (unsigned)( rsp->len ); i++) {
                    DEBUG_DUMP(IOT_LOG_ALL, "0x%02X ", dst[RSP_HDR_SZ + i]);
                }
                DEBUG_DUMP(IOT_LOG_ALL, "\r\n");
            }
            slot->task.total = frame_len; 
            (void)hub_send_tx_task(&slot->task);
        }
    }
}
/* Thread END*/