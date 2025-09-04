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

#define UART_RESP_SLOTS  4 

static TX_QUEUE  s_uart7_q;
static TX_THREAD s_uart7_thread;
static UCHAR     s_uart7_q_buf[32 * sizeof(ULONG)];
static UCHAR     s_uart7_stack[4096];
static volatile UINT s_uart7_started = 0;

static TX_QUEUE  s_uart8_q;
static TX_THREAD s_uart8_thread;
static UCHAR     s_uart8_q_buf[32 * sizeof(ULONG)];
static UCHAR     s_uart8_stack[4096];
static volatile UINT s_uart8_started = 0;

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

static uart_resp_ctx_t s_u7_ctx = {0};
static uart_resp_ctx_t s_u8_ctx = {0};

static VOID uart7_thread_entry(ULONG arg);
static VOID uart8_thread_entry(ULONG arg);

static void uart_tx_done_cb(hub_tx_task_t *t)
{
    if (!t || !t->user_ctx) return;
    uart_resp_slot_t *slot = (uart_resp_slot_t *)t->user_ctx;
    slot->busy = 0;
}

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

void iot_uart_init(void)
{
    memset(&uart7_rx_ring, 0, sizeof(uart7_rx_ring));
    memset(&uart8_rx_ring, 0, sizeof(uart8_rx_ring));
    memset(&uart7_tx_ring, 0, sizeof(uart7_tx_ring));
    memset(&uart8_tx_ring, 0, sizeof(uart8_tx_ring));
    uart7_rx_dma_start();
    uart8_rx_dma_start();
}

static int uart_resp_ctx_init(uart_resp_ctx_t *ctx)
{
    ctx->cap = (uint32_t)(RSP_HDR_SZ + UART_RX_BUF_SZ + 4U);
    for (int i = 0; i < UART_RESP_SLOTS; ++i) {
        ctx->slot[i].frame = (uint8_t*)hub_sdram_alloc_tx(ALIGN32(ctx->cap));
        if (!ctx->slot[i].frame) return 0;
        ctx->slot[i].busy  = 0;
        ctx->slot[i].task.buf         = ctx->slot[i].frame;
        ctx->slot[i].task.total       = 0;
        ctx->slot[i].task.sent        = 0;
        ctx->slot[i].task.stage_tx    = TX_STAGE_HDR;
        ctx->slot[i].task.alloc_flags = HUB_TASK_F_STATIC_BUF | HUB_TASK_F_STATIC_TASK;
        ctx->slot[i].task.done_cb     = uart_tx_done_cb;
        ctx->slot[i].task.user_ctx    = &ctx->slot[i];
    }
    ctx->wr = 0;
    return 1;
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

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart7) {
        uart_rx_drain_dma(huart, UART7_RX, uart7_dma_rx_buf, &uart7_last_pos);
    } else if (huart == &huart8) {
        uart_rx_drain_dma(huart, UART8_RX, uart8_dma_rx_buf, &uart8_last_pos);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart7) {
        uart_rx_drain_dma(huart, UART7_RX, uart7_dma_rx_buf, &uart7_last_pos);
    } else if (huart == &huart8) {
        uart_rx_drain_dma(huart, UART8_RX, uart8_dma_rx_buf, &uart8_last_pos);
    }
}

static inline void uart7_rx_sync_from_dma(void){
    uart_rx_drain_dma(&huart7, UART7_RX, uart7_dma_rx_buf, &uart7_last_pos);
}
static inline void uart8_rx_sync_from_dma(void){
    uart_rx_drain_dma(&huart8, UART8_RX, uart8_dma_rx_buf, &uart8_last_pos);
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
    HAL_UART_Receive_DMA(&huart7, uart7_dma_rx_buf, UART_RX_BUF_SZ);
    __HAL_DMA_ENABLE_IT(huart7.hdmarx, DMA_IT_HT | DMA_IT_TC);
    ((DMA_Stream_TypeDef *)huart7.hdmarx->Instance)->CR |= DMA_SxCR_CIRC;
    uart7_last_pos = dma_get_pos_stable(huart7.hdmarx);
}

uint32_t uart7_rx_available(void){
    uart7_rx_sync_from_dma();
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
    
    HAL_UART_Receive_DMA(&huart8, uart8_dma_rx_buf, UART_RX_BUF_SZ);
    __HAL_DMA_ENABLE_IT(huart8.hdmarx, DMA_IT_HT | DMA_IT_TC);
    ((DMA_Stream_TypeDef *)huart8.hdmarx->Instance)->CR |= DMA_SxCR_CIRC;
    uart8_last_pos = dma_get_pos_stable(huart8.hdmarx);
}

uint32_t uart8_rx_available(void){
    uart8_rx_sync_from_dma(); 
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

static inline int _uart8_poll_fallback(const uint8_t *src, uint32_t len){
    uint32_t n = (len > 64U) ? 64U : len;
    return (HAL_UART_Transmit(&huart8, (uint8_t*)src, n, 100) == HAL_OK);
}

void _uart8_tx_dma_touch(void){
    iot_uart_tx_ring_t *tx = UART8_TX;
    uint32_t w = tx->write_ptr;
    uint32_t r = tx->read_ptr;
    if (tx->busy) {
        HAL_UART_AbortTransmit(&huart8);
        DEBUG_DUMP(IOT_LOG_ERR, "uart8_tx_dma_touch: busy\r\n");
        tx->busy = 0;
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
    if (huart8.hdmatx == NULL) {
        DEBUG_DUMP(IOT_LOG_ERR, "uart8_tx_dma_touch: hdmatx=NULL, len=%lu\r\n", (unsigned long)data_len);
        (void)_uart8_poll_fallback(src, data_len);
        return;
    }
    HAL_StatusTypeDef st = HAL_UART_Transmit_DMA(&huart8, src, data_len);
    if (st == HAL_OK) {
        tx->busy = 1;
        tx->read_ptr = (w > r) ? w : 0;
    } else {
        DEBUG_DUMP(IOT_LOG_ERR, "uart8_tx_dma_touch error: HAL status=%d\r\n", st);
        // tx->busy = 0;
        (void)_uart8_poll_fallback(src, data_len);
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

void uart7_thread_start(void)
{
    if (s_uart7_started) return;
    if (!uart_resp_ctx_init(&s_u7_ctx)) return; /* OOM 保護 */
    tx_queue_create(&s_uart7_q, "uart7_q", TX_1_ULONG, s_uart7_q_buf, sizeof(s_uart7_q_buf));
    tx_thread_create(&s_uart7_thread, "uart7_thread", uart7_thread_entry, 0,
                     s_uart7_stack, sizeof(s_uart7_stack),
                     12, 12, TX_NO_TIME_SLICE, TX_AUTO_START);
    s_uart7_started = 1;
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

int uart7_post_read(uint16_t len, uint32_t data_addr)
{
    if (!s_uart7_started) return 0;
    uart_cmd_msg_t msg = { .type = UART_CMD_READ, .len = len, .data_addr = data_addr };
    return (tx_queue_send(&s_uart7_q, &msg, TX_NO_WAIT) == TX_SUCCESS);
}

int uart8_post_read(uint16_t len, uint32_t data_addr)
{
    if (!s_uart8_started) return 0;
    uart_cmd_msg_t msg = { .type = UART_CMD_READ, .len = len, .data_addr = data_addr };
    return (tx_queue_send(&s_uart8_q, &msg, TX_NO_WAIT) == TX_SUCCESS);
}

static VOID uart7_thread_entry(ULONG arg)
{
    (void)arg;
    for (;;) {
        uart_cmd_msg_t msg;
        if (tx_queue_receive(&s_uart7_q, &msg, TX_WAIT_FOREVER) != TX_SUCCESS) continue;

        if (msg.type == UART_CMD_READ) {
            uart_resp_slot_t *slot = uart_resp_take_free(&s_u7_ctx);
            if (!slot) {
                continue;
            }

            uint8_t *dst = slot->frame;
            hub_rsp_t *rsp = (hub_rsp_t*)dst;

            uint16_t req   = msg.len;                 /* 0 => PEEK */
            uart7_rx_sync_from_dma();
            uint16_t avail = (uint16_t)uart7_rx_available();
            uint16_t need  = (req == 0U) ? 0U : ( (req < avail) ? req : avail );
            uint32_t frame_len = (req == 0U) ? (RSP_HDR_SZ + 4U) : (RSP_HDR_SZ + (uint32_t)req + 4U);

            rsp->status    = HUB_RSP_OK;
            rsp->reserved  = 0;
            rsp->len       = (req == 0U) ? avail : req; 
            rsp->data_addr = msg.data_addr;

            if (req > 0U) {
                uint8_t *payload = dst + RSP_HDR_SZ;
                uint32_t got = 0;
                while (got < need) {
                    uint32_t g = uart7_rx_read(payload + got, (uint32_t)(need - got));
                    got += g;
                    if (g == 0U) break;
                }
                if (need < req) {
                    memset(payload + need, HUB_ERR_FILL_BYTE, (size_t)(req - need));
                }
            }
            uint32_t crc = iot_hub_crc32_hard(dst, frame_len - 4U);
            memcpy(dst + frame_len - 4U, &crc, 4U);

            slot->task.total = frame_len;
            (void)hub_send_tx_task(&slot->task);
        }
    }
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

            uint8_t *dst = slot->frame;
            hub_rsp_t *rsp = (hub_rsp_t*)dst;

            uint16_t req   = msg.len;                 /* 0 => PEEK */
            uart8_rx_sync_from_dma(); 
            uint16_t avail = (uint16_t)uart8_rx_available();
            uint16_t need  = (req == 0U) ? 0U : ( (req < avail) ? req : avail );
            uint32_t frame_len = (req == 0U) ? (RSP_HDR_SZ + 4U) : (RSP_HDR_SZ + (uint32_t)req + 4U);

            rsp->status    = HUB_RSP_OK;
            rsp->reserved  = 0;
            rsp->len       = (req == 0U) ? avail : req;
            rsp->data_addr = msg.data_addr;

            if (req > 0U) {
                uint8_t *payload = dst + RSP_HDR_SZ;
                uint32_t got = 0;
                while (got < need) {
                    uint32_t g = uart8_rx_read(payload + got, (uint32_t)(need - got));
                    got += g;
                    if (g == 0U) break;
                }
                if (need < req) {
                    memset(payload + need, HUB_ERR_FILL_BYTE, (size_t)(req - need));
                }
            }

            uint32_t crc = iot_hub_crc32_hard(dst, frame_len - 4U);
            memcpy(dst + frame_len - 4U, &crc, 4U);

            slot->task.total = frame_len;
            (void)hub_send_tx_task(&slot->task);
        }
    }
}