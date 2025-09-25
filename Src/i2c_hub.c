#include "i2c_hub.h"
#include "iot.h"
#if IOT_HUB_GPIO
#include "iot_util.h"
#endif
#include <string.h>
#include <stddef.h> 
#include <stdint.h>
#include "usart.h"
#include "i2c.h"
#include "tx_api.h"
#include "dac.h"
#include "adc.h"
#include "crc.h"
#include "i2c_hub_uart.h"
#include "i2c_hub_modbus_server.h"
#include "i2c_hub_filex.h"
#include "modbus_formosa.h"

static TX_QUEUE cmd_queue;
static TX_QUEUE rsp_queue;
static TX_QUEUE i2c_tx_stage_q;
static TX_MUTEX crc_mutex;
static TX_THREAD worker_thread;

static VOID worker_thread_entry(ULONG arg);

static uint8_t rx_hdr[CMD_HDR_SZ] __attribute__((aligned(32)));
static hub_cmd_t *rx_hdr_ptr = (hub_cmd_t *)rx_hdr;

static uint8_t  tx_hdr[RSP_HDR_SZ]  __attribute__((aligned(32)));
static hub_rsp_t *tx_hdr_ptr = (hub_rsp_t *)tx_hdr;

static volatile hub_tx_task_t *cur_tx;
static volatile hub_rx_task_t cur_rx = {
    .buf      = NULL,
    .total    = 0,
    .sent     = 0,
    .stage_rx = RX_STAGE_HDR,
};

static uint8_t s_rsp_ok[RSP_HDR_SZ + 4] __attribute__((aligned(32))); /* return unknown target frame */
static uint8_t s_unknown_target[RSP_HDR_SZ + 4] __attribute__((aligned(32))); /* return unknown target frame */
static uint8_t s_unknown_cmd[RSP_HDR_SZ + 4] __attribute__((aligned(32))); /* return unknown cmd frame */
static uint8_t s_err_mem[RSP_HDR_SZ + 4] __attribute__((aligned(32))); /* return error mem */
static uint8_t s_err_crc[RSP_HDR_SZ + 4] __attribute__((aligned(32))); /* return error crc32 frame */
static uint8_t s_busy_frame[RSP_HDR_SZ + 4] __attribute__((aligned(32))); /* return busy frame */
static uint8_t s_worker_ack_frame[RSP_HDR_SZ + 4] __attribute__((aligned(32))); /* return worker ack frame */
static uint8_t s_err_none[RSP_HDR_SZ + 4] __attribute__((aligned(32))); /* return error none frame */

static inline int hub_tx_buf_is_from_arena(const void *buf, uint32_t len)
{
    if (!g_hub_tx_arena.base || !g_hub_tx_arena.size) return 0;
    uintptr_t start = (uintptr_t)buf;
    uintptr_t end   = start + len;
    uintptr_t arena_start = (uintptr_t)g_hub_tx_arena.base;
    uintptr_t arena_end   = arena_start + g_hub_tx_arena.size;
    return (start >= arena_start) && (end <= arena_end);
}

/* Transacion Init */
static volatile rx_txn_t g_rx_txn = {0};
static volatile hub_rd_mode_t g_rd_mode = RD_IDLE;
static volatile uint8_t g_tx_busy = 0;

/* Start RX TX Transaction commit section */
static inline void rx_txn_begin(uint8_t *p, uint32_t total, uint32_t mark)
{
    g_rx_txn.ptr       = p;
    g_rx_txn.total     = total;
    g_rx_txn.head_mark = mark;
    g_rx_txn.active    = 1;
}

static inline void rx_txn_rollback(const char *reason)
{
    // (void)reason;
    DEBUG_DUMP(IOT_LOG_ALL, "RX TXN ROLLBACK: %s\r\n", reason);
    if (g_rx_txn.active && g_rx_txn.ptr) {
        hub_spsc_undo_alloc_to(&g_hub_rx_arena, g_rx_txn.head_mark);
    }
    g_rx_txn.ptr      = NULL;
    g_rx_txn.total    = 0;
    g_rx_txn.head_mark= 0;
    g_rx_txn.active   = 0;

    cur_rx.stage_rx  = RX_STAGE_HDR;
    cur_rx.buf       = NULL;
    cur_rx.total     = 0;
    cur_rx.sent      = 0;
}

static inline void tx_cancel_in_isr(void)
{
    if (g_tx_busy > 0){
        g_tx_busy -= 1;
    }
    if (cur_tx) {
        /* flag the task is reuseable*/
        if (cur_tx->done_cb) cur_tx->done_cb((hub_tx_task_t *)cur_tx);

        uint32_t full_len = ALIGN32(cur_tx->total);
        if (!(cur_tx->alloc_flags & HUB_TASK_F_STATIC_BUF)) {
            hub_sdram_free_tx((void*)cur_tx->buf, full_len);
        }
        if (!(cur_tx->alloc_flags & HUB_TASK_F_STATIC_TASK)) {
            hub_sdram_free_tx((void*)cur_tx, ALIGN32(sizeof(hub_tx_task_t)));
        }
        cur_tx = NULL;
    }
}
/* End RX TX Transaction commit section */
/* Start I2C HAL Address Callback */
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t dir, uint16_t addr7)
{
    if (hi2c != &hi2c2 || addr7 != HUB_I2C_ADDR) return;
    if (dir == I2C_DIRECTION_TRANSMIT)      /* Master to Slave */
    {
        HAL_I2C_Slave_Seq_Receive_DMA(hi2c, rx_hdr, CMD_HDR_SZ, I2C_LAST_FRAME);
    }else if (dir == I2C_DIRECTION_RECEIVE) {
        switch (g_rd_mode) {
            case RD_PEEK: {
                if (!cur_tx) {
                    if (tx_queue_receive(&i2c_tx_stage_q, &cur_tx, TX_NO_WAIT) != TX_SUCCESS) {
                        dcache_clean32_range(s_busy_frame, sizeof(s_busy_frame));
                        HAL_I2C_Slave_Seq_Transmit_DMA(hi2c, s_busy_frame, sizeof(s_busy_frame), I2C_LAST_FRAME);
                        return;
                    }
                }
                dcache_clean32_range(cur_tx->buf, cur_tx->total);
                cur_tx->sent = cur_tx->total;
                HAL_I2C_Slave_Seq_Transmit_DMA(hi2c, cur_tx->buf, cur_tx->total, I2C_LAST_FRAME);
                g_rd_mode = RD_IDLE;
                return;
            }

            case RD_DATA_WAIT: {
                if (!cur_tx) {
                    if (tx_queue_receive(&i2c_tx_stage_q, &cur_tx, TX_NO_WAIT) != TX_SUCCESS) {
                        dcache_clean32_range(s_busy_frame, sizeof(s_busy_frame));
                        HAL_I2C_Slave_Seq_Transmit_DMA(hi2c, s_busy_frame, sizeof(s_busy_frame), I2C_LAST_FRAME);
                        return;
                    }
                }
                g_tx_busy = 1;
                dcache_clean32_range(s_worker_ack_frame, sizeof(s_worker_ack_frame));
                HAL_I2C_Slave_Seq_Transmit_DMA(hi2c, s_worker_ack_frame, sizeof(s_worker_ack_frame), I2C_LAST_FRAME);
                g_rd_mode = RD_DATA_READY_PAY;
                return;
            }

            case RD_DATA_READY_PAY: {
                if (!cur_tx) {
                    DEBUG_DUMP(IOT_LOG_ERR, "Deadpool Need to Reboot....\r\n");
                    dcache_clean32_range(s_busy_frame, sizeof(s_busy_frame));
                    HAL_I2C_Slave_Seq_Transmit_DMA(hi2c, s_busy_frame, sizeof(s_busy_frame), I2C_LAST_FRAME);
                    return;
                }
                uint16_t first = (cur_tx->stage_tx == TX_STAGE_HDR) ? RSP_HDR_SZ + 4 : cur_tx->total ;
                dcache_clean32_range(cur_tx->buf, first);
                HAL_I2C_Slave_Seq_Transmit_DMA(hi2c, cur_tx->buf, first, I2C_FIRST_FRAME);
                cur_tx->sent = first;
                return;
            }

            default: {
                dcache_clean32_range(s_busy_frame, sizeof(s_busy_frame));
                HAL_I2C_Slave_Seq_Transmit_DMA(hi2c, s_busy_frame, sizeof(s_busy_frame), I2C_LAST_FRAME);
                return;
            }
        }
    }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c != &hi2c2) return;
    if (((uintptr_t)g_hub_rx_arena.base & 0xF0000000u) != 0xC0000000u) {
        DEBUG_DUMP(IOT_LOG_ERR, "RX ARENA CORRUPTED! base=%p\r\n", g_hub_rx_arena.base);
    }
    if (cur_rx.stage_rx == RX_STAGE_HDR) {
        /* Header */
        dcache_invalidate32_range(rx_hdr, CMD_HDR_SZ);
        if (rx_hdr_ptr->operation == HUB_OP_READ) {
            if (rx_hdr_ptr->len == 0) {
                g_rd_mode = RD_PEEK;
            } else {
                g_rd_mode = RD_DATA_WAIT;
            }
        }
        uint32_t rx_plen = ((uint32_t)rx_hdr_ptr->len + 4U);
        uint32_t rx_head_mark = hub_spsc_mark_head(&g_hub_rx_arena);
        uint32_t frame_total  = CMD_HDR_SZ + rx_plen;

        if (frame_total > HUB_I2C_RX_MAX_FRAME) {
            DEBUG_DUMP(IOT_LOG_ERR, "I2C RX: payload too large %lu > %u → BUSY\r\n",
                    (unsigned long)frame_total, (unsigned)HUB_I2C_RX_MAX_FRAME);
            dcache_clean32_range(s_busy_frame, sizeof(s_busy_frame));
            HAL_I2C_Slave_Seq_Transmit_DMA(hi2c, s_busy_frame, sizeof(s_busy_frame), I2C_LAST_FRAME);
            HAL_I2C_EnableListen_IT(hi2c);
            return;
        }

        uint8_t *rx_frame_ptr = (uint8_t*)hub_sdram_alloc_rx(frame_total);
        if (rx_frame_ptr == NULL) {
            DEBUG_DUMP(IOT_LOG_ERR, "I2C2 RX: hub_sdram_alloc_rx failed (need=%lu)\r\n", (unsigned long)frame_total);
            dcache_clean32_range(s_err_mem, sizeof(s_err_mem));
            HAL_I2C_Slave_Seq_Transmit_DMA(hi2c, s_err_mem, sizeof(s_err_mem), I2C_LAST_FRAME);
            HAL_I2C_EnableListen_IT(hi2c);
            return;
        }

        dcache_invalidate32_range(rx_frame_ptr, frame_total);
        rx_txn_begin(rx_frame_ptr, frame_total, rx_head_mark);
        memcpy(rx_frame_ptr, rx_hdr, CMD_HDR_SZ);
        cur_rx.buf      = rx_frame_ptr;
        cur_rx.total    = frame_total;
        cur_rx.sent     = 0;
        cur_rx.stage_rx = RX_STAGE_PAY;
        uint8_t *payload_dst = rx_frame_ptr + CMD_HDR_SZ;

        HAL_I2C_Slave_Seq_Receive_DMA(hi2c, payload_dst, (uint16_t)rx_plen, I2C_LAST_FRAME);
        return; 
    }
    if (cur_rx.stage_rx == RX_STAGE_PAY) {
        /* payload+CRC */
        uint32_t rx_plen = cur_rx.total - CMD_HDR_SZ;
        dcache_invalidate32_range(cur_rx.buf + CMD_HDR_SZ, rx_plen);
        uint32_t crc_rx;
        memcpy(&crc_rx, cur_rx.buf + CMD_HDR_SZ + rx_plen - 4U, 4U);

        uint32_t cmd_crc_calc = iot_hub_crc32_hard(cur_rx.buf, cur_rx.total - 4U);
        if (cmd_crc_calc != crc_rx) {
            DEBUG_DUMP(IOT_LOG_ERR, "HUB ERROR: Crc mismatch: %08lX != %08lX\r\n", cmd_crc_calc, crc_rx);
            rx_txn_rollback("crc");
            cur_rx.buf      = NULL;
            cur_rx.total    = 0;
            cur_rx.sent     = 0;
            cur_rx.stage_rx = RX_STAGE_HDR;
            dcache_clean32_range(s_err_crc, sizeof(s_err_crc));
            HAL_I2C_Slave_Seq_Transmit_DMA(hi2c, s_err_crc, sizeof(s_err_crc), I2C_LAST_FRAME);
            HAL_I2C_EnableListen_IT(hi2c);
            return;
        }
        hub_cmd_t *cmd_ptr = (hub_cmd_t *)cur_rx.buf;
        if (tx_queue_send(&cmd_queue, &cmd_ptr, TX_NO_WAIT) != TX_SUCCESS) {
            DEBUG_DUMP(IOT_LOG_ERR, "HAL_I2C_SlaveRxCpltCallback: cmd_queue full\r\n");
            rx_txn_rollback("queue_full");
            dcache_clean32_range(s_busy_frame, sizeof(s_busy_frame));
            HAL_I2C_Slave_Seq_Transmit_DMA(hi2c, s_busy_frame, sizeof(s_busy_frame), I2C_LAST_FRAME);
            HAL_I2C_EnableListen_IT(hi2c);
            return;
        }
        
        g_rx_txn.active = 0;
        /* reset cur_rx */
        cur_rx.buf      = NULL;
        cur_rx.total    = 0;
        cur_rx.sent     = 0;
        cur_rx.stage_rx = RX_STAGE_HDR;
        HAL_I2C_EnableListen_IT(hi2c);
        return;
    }
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c != &hi2c2) return;
    if (!cur_tx || g_tx_busy != 1) {
        HAL_I2C_EnableListen_IT(hi2c);
        return;
    }
    if (cur_tx->sent < cur_tx->total && cur_tx->sent != 0) {
        uint16_t remain = cur_tx->total - cur_tx->sent;
        uint8_t *next   = cur_tx->buf + cur_tx->sent;
        uint16_t chunk  = (remain > I2C_SIZE_MTU) ? I2C_SIZE_MTU : (uint16_t)remain;
        dcache_clean32_range(next, chunk);
        uint32_t option = (remain == chunk) ? I2C_LAST_FRAME : I2C_NEXT_FRAME;
        HAL_I2C_Slave_Seq_Transmit_DMA(hi2c, next, chunk, option);
        cur_tx->sent += chunk;
        return;
    }
    if ((cur_tx->sent >= cur_tx->total || cur_tx->total == 0 ) && g_rd_mode != RD_IDLE) {
        /* done tx, set flag reusable */
        if (cur_tx->done_cb) cur_tx->done_cb((hub_tx_task_t *)cur_tx);
        uint32_t full_len = ALIGN32(cur_tx->total);
        if (!(cur_tx->alloc_flags & HUB_TASK_F_STATIC_BUF)) {
            hub_sdram_free_tx((void*)cur_tx->buf, full_len);
        }
        if (!(cur_tx->alloc_flags & HUB_TASK_F_STATIC_TASK)) {
            hub_sdram_free_tx((void*)cur_tx, ALIGN32(sizeof(hub_tx_task_t)));
        }
        cur_tx = NULL;
        g_tx_busy = 0;
        if (g_rd_mode == RD_DATA_READY_PAY) g_rd_mode = RD_IDLE;
    }
    HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c != &hi2c2) return;
    if (cur_rx.stage_rx == RX_STAGE_PAY || g_rx_txn.active) {
        HAL_I2C_EnableListen_IT(hi2c);
        return;
    }

    if (g_tx_busy != 1){
        tx_cancel_in_isr();
    }

    HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c != &hi2c2) return;
    CLEAR_BIT(hi2c->Instance->CR1, I2C_CR1_RXDMAEN | I2C_CR1_TXDMAEN);
    if (hi2c->hdmarx && HAL_DMA_GetState(hi2c->hdmarx) != HAL_DMA_STATE_READY) {
        (void)HAL_DMA_Abort(hi2c->hdmarx);
    }
    if (hi2c->hdmatx && HAL_DMA_GetState(hi2c->hdmatx) != HAL_DMA_STATE_READY) {
        (void)HAL_DMA_Abort(hi2c->hdmatx);
    }
    uint32_t err_flags = I2C_FLAG_BERR | I2C_FLAG_ARLO | I2C_FLAG_OVR |
                         I2C_FLAG_PECERR | I2C_FLAG_TIMEOUT | I2C_FLAG_AF |
                         I2C_FLAG_STOPF;
    __HAL_I2C_CLEAR_FLAG(hi2c, err_flags);

    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_ADDR);
    if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_TXIS) != RESET)
    {
        hi2c->Instance->TXDR = 0x00U;
    }
    /* Flush TX register if not empty */
    if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_TXE) == RESET)
    {
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_TXE);
    }
    rx_txn_rollback("ER");
    tx_cancel_in_isr();
    HAL_I2C_EnableListen_IT(hi2c);
}
/* End I2C HAL Address Callback */
/* Start I2C HUB tools */
int hub_send_tx_frame(uint8_t *buf, uint32_t total)
{
    uint32_t full_len = ALIGN32(total);
    int buf_is_dyn = hub_tx_buf_is_from_arena(buf, full_len);

    hub_tx_task_t *task = (hub_tx_task_t*)hub_sdram_alloc_tx(ALIGN32(sizeof(hub_tx_task_t)));
    if (!task) {
        if (buf_is_dyn) {
            hub_sdram_free_tx(buf, full_len);
        }
        return 0;
    }
    dcache_clean32_range(buf, total);
    task->buf        = buf;
    task->total      = total;
    task->sent       = 0;
    task->stage_tx   = TX_STAGE_HDR;
    task->alloc_flags= buf_is_dyn ? 0u : HUB_TASK_F_STATIC_BUF;
    task->done_cb    = NULL;
    task->user_ctx   = NULL;

    if (tx_queue_send(&i2c_tx_stage_q, &task, TX_NO_WAIT) != TX_SUCCESS) {
        if (buf_is_dyn) {
            hub_sdram_free_tx(buf, full_len);
        }
        hub_sdram_free_tx(task, ALIGN32(sizeof(hub_tx_task_t)));
        return 0;
    }
    return 1;
}

int hub_gen2send_tx_frame(uint8_t *data, uint32_t length)
{
    uint32_t frame_len = RSP_HDR_SZ + length + 4U;
    uint8_t *tx_frame_ptr = (uint8_t*)hub_sdram_alloc_tx(ALIGN32(frame_len));

    memcpy(tx_frame_ptr + RSP_HDR_SZ , data, length);
    dcache_invalidate32_range(tx_frame_ptr, ALIGN32(frame_len));
    if (tx_frame_ptr) {
        hub_rsp_t *rsp = (hub_rsp_t*)tx_frame_ptr;
        rsp->status    = HUB_RSP_OK;
        rsp->reserved  = 0;
        rsp->len       = length;
        rsp->data_addr = 0;
        uint32_t crc = iot_hub_crc32_hard(tx_frame_ptr, RSP_HDR_SZ + length);
        memcpy(tx_frame_ptr + RSP_HDR_SZ + length, &crc, 4U);
        dcache_clean32_range(tx_frame_ptr, frame_len);
    }
    return hub_send_tx_frame(tx_frame_ptr, ALIGN32(frame_len));
}

int hub_send_tx_task(hub_tx_task_t *task)
{
    dcache_clean32_range(task->buf, task->total);
    task->sent     = 0;
    task->stage_tx = TX_STAGE_HDR;

    if (tx_queue_send(&i2c_tx_stage_q, &task, TX_NO_WAIT) != TX_SUCCESS) {
        return 0;
    }
    return 1;
}

void hub_send_tx_flush(void)
{
    __disable_irq();
    if (cur_tx) {
        if (cur_tx->done_cb) cur_tx->done_cb((hub_tx_task_t *)cur_tx);
        uint32_t full_len = ALIGN32(cur_tx->total);
        if (!(cur_tx->alloc_flags & HUB_TASK_F_STATIC_BUF))
            hub_sdram_free_tx((void *)cur_tx->buf, full_len);
        if (!(cur_tx->alloc_flags & HUB_TASK_F_STATIC_TASK))
            hub_sdram_free_tx((void*)cur_tx, ALIGN32(sizeof(hub_tx_task_t)));
        cur_tx = NULL;
    }
    g_tx_busy = 0;

    hub_tx_task_t *task;
    while (tx_queue_receive(&i2c_tx_stage_q, &task, TX_NO_WAIT) == TX_SUCCESS) {
        if (task->done_cb) task->done_cb((hub_tx_task_t *)task);
        uint32_t full_len = ALIGN32(task->total);
        if (!(task->alloc_flags & HUB_TASK_F_STATIC_BUF))
            hub_sdram_free_tx(task->buf, full_len);
        if (!(task->alloc_flags & HUB_TASK_F_STATIC_TASK))
            hub_sdram_free_tx(task, ALIGN32(sizeof(hub_tx_task_t)));
    }
    __enable_irq();
}

void Init_I2C_HUB_QUIC_RET(void){
    /* HUB_RSP_OK -> 0*/
    hub_rsp_t *br_ok = (hub_rsp_t*)s_rsp_ok;
    br_ok->status    = HUB_RSP_OK;
    br_ok->reserved  = 0;
    br_ok->len       = 0;
    br_ok->data_addr = 0;
    uint32_t crc = iot_hub_crc32_hard(s_rsp_ok, RSP_HDR_SZ);
    memcpy(s_rsp_ok + RSP_HDR_SZ, &crc, 4);
    dcache_clean32_range(s_rsp_ok, sizeof(s_rsp_ok));
    /* HUB_RSP_ERR_UNKNOWN_TARGET -> 1 */
    hub_rsp_t *br_unknown_target = (hub_rsp_t*)s_unknown_target;
    br_unknown_target->status    = HUB_RSP_ERR_UNKNOWN_TARGET;
    br_unknown_target->reserved  = 0;
    br_unknown_target->len       = 0;
    br_unknown_target->data_addr = 0;
    crc = iot_hub_crc32_hard(s_unknown_target, RSP_HDR_SZ);
    memcpy(s_unknown_target + RSP_HDR_SZ, &crc, 4);
    dcache_clean32_range(s_unknown_target, sizeof(s_unknown_target));
    /* HUB_RSP_ERR_UNKNOWN_CMD -> 2 */
    hub_rsp_t *br_unknown_cmd = (hub_rsp_t*)s_unknown_cmd;
    br_unknown_cmd->status    = HUB_RSP_ERR_UNKNOWN_CMD;
    br_unknown_cmd->reserved  = 0;
    br_unknown_cmd->len       = 0;
    br_unknown_cmd->data_addr = 0;
    crc = iot_hub_crc32_hard(s_unknown_cmd, RSP_HDR_SZ);
    memcpy(s_unknown_cmd + RSP_HDR_SZ, &crc, 4);
    dcache_clean32_range(s_unknown_cmd, sizeof(s_unknown_cmd));
    /* HUB_RSP_ERR_MEMORY -> 3 */
    hub_rsp_t *br_err_mem = (hub_rsp_t*)s_err_mem;
    br_err_mem->status    = HUB_RSP_ERR_MEMORY;
    br_err_mem->reserved  = 0;
    br_err_mem->len       = 0;
    br_err_mem->data_addr = 0;
    crc = iot_hub_crc32_hard(s_err_mem, RSP_HDR_SZ);
    memcpy(s_err_mem + RSP_HDR_SZ, &crc, 4);
    dcache_clean32_range(s_err_mem, sizeof(s_err_mem));
    /* HUB_RSP_ERR_CRC -> 4 */
    hub_rsp_t *br_err_crc = (hub_rsp_t*)s_err_crc;
    br_err_crc->status    = HUB_RSP_ERR_CRC;
    br_err_crc->reserved  = 0;
    br_err_crc->len       = 0;
    br_err_crc->data_addr = 0;
    crc = iot_hub_crc32_hard(s_err_crc, RSP_HDR_SZ);
    memcpy(s_err_crc + RSP_HDR_SZ, &crc, 4);
    dcache_clean32_range(s_err_crc, sizeof(s_err_crc));
    /* HUB_RSP_BUSY -> 5 */
    hub_rsp_t *br_busy = (hub_rsp_t*)s_busy_frame;
    br_busy->status    = HUB_RSP_BUSY;
    br_busy->reserved  = 0;
    br_busy->len       = 0;
    br_busy->data_addr = 0;
    crc = iot_hub_crc32_hard(s_busy_frame, RSP_HDR_SZ);
    memcpy(s_busy_frame + RSP_HDR_SZ, &crc, 4);
    dcache_clean32_range(s_busy_frame, sizeof(s_busy_frame));
    hub_rsp_t *br_worker_ack = (hub_rsp_t*)s_worker_ack_frame;
    br_worker_ack->status    = HUB_RSP_WORKER_ACK;
    br_worker_ack->reserved  = 0;
    br_worker_ack->len       = 0;
    br_worker_ack->data_addr = 0;
    crc = iot_hub_crc32_hard(s_worker_ack_frame, RSP_HDR_SZ);
    memcpy(s_worker_ack_frame + RSP_HDR_SZ, &crc, 4);
    dcache_clean32_range(s_worker_ack_frame, sizeof(s_worker_ack_frame));
    /* HUB_RSP_ERR_NONE -> 7 */
    hub_rsp_t *br_err_none = (hub_rsp_t*)s_err_none;
    br_err_none->status    = HUB_RSP_ERR_NONE;
    br_err_none->reserved  = 0;
    br_err_none->len       = 0;
    br_err_none->data_addr = 0;
    crc = iot_hub_crc32_hard(s_err_none, RSP_HDR_SZ);
    memcpy(s_err_none + RSP_HDR_SZ, &crc, 4);
    dcache_clean32_range(s_err_none, sizeof(s_err_none));
}

/* End I2C HUB tools */

/* Start I2C HUB worker thread */
void iot_hub_start(void)
{
    /*Init Dummy Frame */
    static ULONG cmd_queue_buf   [QUEUE_LEN        * MSG_WORDS_PTR];
    static ULONG rsp_queue_buf   [QUEUE_LEN        * MSG_WORDS_PTR];
    static ULONG tx_stage_buf    [I2C_STAGE_Q_LEN  * MSG_WORDS_PTR];
    static UCHAR worker_stack[16384];
    Init_I2C_HUB_QUIC_RET();

    UINT s;
    s = tx_queue_create(&cmd_queue, "cmd_queue", MSG_WORDS_PTR, cmd_queue_buf, QSIZE(cmd_queue_buf));
    assert_param(s == TX_SUCCESS);
    s = tx_queue_create(&rsp_queue, "rsp_queue", MSG_WORDS_PTR, rsp_queue_buf, QSIZE(rsp_queue_buf));
    assert_param(s == TX_SUCCESS);
    s = tx_queue_create(&i2c_tx_stage_q,  "i2c_tx_stage_q",  MSG_WORDS_PTR, tx_stage_buf,  QSIZE(tx_stage_buf));
    assert_param(s == TX_SUCCESS);
    (void)s;
    tx_mutex_create(&crc_mutex, "crc_mutex", TX_INHERIT);
    tx_thread_create(&worker_thread, "hub_worker",
                 worker_thread_entry, 0,
                 worker_stack, sizeof(worker_stack),
                 10, 10, TX_NO_TIME_SLICE, TX_DONT_START);
    hub_mem_init();
    // iot_uart_init();
    HAL_I2C_EnableListen_IT(&hi2c2);
    tx_thread_resume(&worker_thread);
}

static VOID worker_thread_entry(ULONG arg)
{
    (void)arg;
    for (;;) {
        uint8_t *tx_frame_ptr = NULL;
        hub_cmd_t *cmd = NULL;
        tx_queue_receive(&cmd_queue, &cmd, TX_WAIT_FOREVER);
        uint8_t           target    = cmd->target;
        uint8_t*          payload   = cmd->payload;
        hub_operation_t   op        = cmd->operation;
        uint16_t          in_len    = cmd->len;
        uint32_t          data_addr = cmd->data_addr;
        uint32_t rx_total = ALIGN32(CMD_HDR_SZ + ((uint32_t)in_len + 4U));
        if (op == HUB_OP_READ) {
            hub_sdram_free_rx(cmd, rx_total);
            cmd = NULL;
        }
        /* CONTROLL REPLY*/
        switch (target) {
            case HUB_TARGET_UART:
                if (op == HUB_OP_CONFIG) {
                    // uart8_thread_start();
                    hub_uart_cfg cfg;
                    memset(&cfg, 0, sizeof(hub_uart_cfg));
                    memcpy(&cfg, cmd->payload, sizeof(hub_uart_cfg) < in_len ? sizeof(hub_uart_cfg) : in_len);
                    DEBUG_DUMP(IOT_LOG_INFO, "HUB_TARGET_UART.HUB_OP_CONFIG Type:%d Name: %d\r\n", cfg.type, cfg.name);
                    switch (cfg.type) {
                        case hub_config_rs485:
                            if (cfg.name == hub_config_baudrate) {
                                uint32_t baud = 0;
                                memcpy(&baud, cfg.values, sizeof(uint32_t));
                                
                                modbus_thread_stop();
                                MX_UART8_DeInit();
                                MX_UART8_Init_WithBaud(baud);
                                modbus_thread_start();
                            }
                            break;
                        case hub_config_modbus:
                            if (cfg.name == hub_config_thread_run) {
                                uint8_t value = 0;
                                memcpy(&value, cfg.values, sizeof(uint8_t));
                                if (value == 0)
                                    modbus_thread_stop();
                                else
                                    modbus_thread_start();
                            }
                            else if (cfg.name == hub_config_ups120) {
                                uint8_t value = 0;
                                memcpy(&value, cfg.values, sizeof(uint8_t));
                                formosa_set_ups120(value);
                            }
                            else if (cfg.name == hub_config_init_device) {
                                formosa_init_setting();
                            }
                            else if (cfg.name == hub_config_slave_addr) {
                                formosa_set_slave_addr(cfg.values);
                            }
                            else if (cfg.name == hub_config_decice_qty) {
                                formosa_set_qty(cfg.values);
                            }
                            break;
                    }
                    if (cmd) { hub_sdram_free_rx((void*)cmd, rx_total); cmd = NULL; }
                    goto _continue_loop_;
                    // tx_hdr_ptr->status = HUB_RSP_OK;
                } else if (op == HUB_OP_WRITE && in_len > 0) {
                    DEBUG_DUMP(IOT_LOG_ALL, "HUB_TARGET_UART: Sending %d bytes to UART8\r\n", in_len);
                    iot_uart8_tx_write((uint8_t *)payload, in_len);
                    if (cmd) { hub_sdram_free_rx((void*)cmd, rx_total); cmd = NULL; }
                    goto _continue_loop_;
                } else if (op == HUB_OP_READ) {
                    // DEBUG_DUMP(IOT_LOG_ALL, "HUB_TARGET_UART: Receiving %d bytes from UART8\r\n", in_len);
                    // (void)uart8_post_read(in_len, data_addr);
                    hub_uart_read rd;
                    memset(&rd, 0, sizeof(hub_uart_read));
                    memcpy(&rd, payload, sizeof(hub_uart_read) < in_len ? sizeof(hub_uart_read) : in_len);
                    DEBUG_DUMP(IOT_LOG_INFO, "HUB_TARGET_UART.HUB_OP_READ Type:%d, Num: %d\r\n", rd.type, rd.num);
                    switch (rd.type) {
                        case hub_read_length:
                            for (int i = 0; i < formosa_device_count; i++) {
                                if (rd.num == formosa_setting[i].slave_addr) {
                                    uint16_t data_len = 0;
                                    data_len = formosa_setting[i].summamry_len + formosa_setting[i].detail_len;
                                    hub_gen2send_tx_frame(&data_len, 2);
                                    break;
                                }
                            }
                            break;
                        case hub_read_content:
                            for (int i = 0; i < formosa_device_count; i++) {
                                if (rd.num == formosa_setting[i].slave_addr) {
                                    uint16_t data_len = 0;
                                    data_len = formosa_setting[i].summamry_len + formosa_setting[i].detail_len;
                                    uint8_t data_content[data_len];
                                    memcpy(data_content, formosa_setting[i].summamry, formosa_setting[i].summamry_len);
                                    memcpy(data_content + formosa_setting[i].summamry_len, formosa_setting[i].detail, formosa_setting[i].detail_len);
                                    hub_gen2send_tx_frame(data_content, data_len);
                                    break;
                                }
                            }
                            break;
                    }

                    goto _continue_loop_;
                } else {
                    DEBUG_DUMP(IOT_LOG_ERR, "HUB_TARGET_UART: Unknown operation %d\r\n", op);
                    // hub_send_tx_frame(s_unknown_cmd, sizeof(s_unknown_cmd));
                    if (cmd) { hub_sdram_free_rx((void*)cmd, rx_total); cmd = NULL; }
                    goto _continue_loop_;
                }
                break;
#if IOT_HUB_GPIO
            case HUB_TARGET_GPIO:{
                switch (op) {
                    case HUB_OP_CONFIG: {
                        DEBUG_DUMP(IOT_LOG_DEBUG, "HUB_TARGET_GPIO: Configuring pin_code=0x%04X, cfg=%d, af=%d\r\n",
                                   ((hub_cfg_payload_t *)data_addr)->pin_code,
                                   ((hub_cfg_payload_t *)data_addr)->cfg,
                                   ((hub_cfg_payload_t *)data_addr)->af);
                        hub_cfg_payload_t *cfg = (hub_cfg_payload_t *)data_addr;
                        hub_apply_gpio_cfg(cfg);
                        if (cmd) { hub_sdram_free_rx((void*)cmd, rx_total); cmd = NULL; }
                        goto _continue_loop_;  
                    }
                    case HUB_OP_WRITE: {
                        DEBUG_DUMP(IOT_LOG_DEBUG, "HUB_TARGET_GPIO: Writing pin_code=0x%04X, value=%d\r\n",
                                   *(uint16_t *)data_addr, *(uint8_t *)(data_addr + 2));
                        uint8_t *val = (uint8_t *)data_addr;
                        GPIO_TypeDef *port = HUB_GET_PORT(*(uint16_t *)val);
                        HAL_GPIO_WritePin(port, HUB_GET_PIN(*(uint16_t *)val), (*(val+2) ? GPIO_PIN_SET : GPIO_PIN_RESET));
                        if (cmd) { hub_sdram_free_rx((void*)cmd, rx_total); cmd = NULL; }
                        goto _continue_loop_;  
                    }
                    case HUB_OP_READ: {
                        DEBUG_DUMP(IOT_LOG_DEBUG, "HUB_TARGET_GPIO: Reading pin_code=0x%04X\r\n",
                                   *(uint16_t *)data_addr);
                        /* temp reply */
                        hub_send_tx_frame(s_rsp_ok, sizeof(s_rsp_ok));
                        /* temp reply */
                        goto _continue_loop_;  
                    }
                    default:{
                        DEBUG_DUMP(IOT_LOG_ERR, "HUB_TARGET_GPIO: Unknown operation %d\r\n", op);
                        // hub_send_tx_frame(s_unknown_cmd, sizeof(s_unknown_cmd));
                        if (cmd) { hub_sdram_free_rx((void*)cmd, rx_total); cmd = NULL; }
                        goto _continue_loop_;  
                    }
                }
                break;
            }
#endif
            case HUB_TARGET_SPI:{ /*w5500_modbus_server_helper*/
                if (op == HUB_OP_WRITE) {
                    w5500_modbus_server_helper();
                    if (cmd) { hub_sdram_free_rx((void*)cmd, rx_total); cmd = NULL; }
                    goto _continue_loop_;
                } else {
                    
                    DEBUG_DUMP(IOT_LOG_ERR, "HUB_TARGET_SPI: Unknown operation %d\r\n", op);
                    // hub_send_tx_frame(s_unknown_cmd, sizeof(s_unknown_cmd));
                    if (cmd) { hub_sdram_free_rx((void*)cmd, rx_total); cmd = NULL; }
                    goto _continue_loop_;
                }
                break;
            }
#if IOT_HUB_I2C
            case HUB_TARGET_I2C:{
                if (op == HUB_OP_WRITE) {
                    hub_send_tx_frame(s_rsp_ok, sizeof(s_rsp_ok));
                    if (cmd) { hub_sdram_free_rx((void*)cmd, rx_total); cmd = NULL; }
                    goto _continue_loop_;
                } else {
                    DEBUG_DUMP(IOT_LOG_ERR, "HUB_TARGET_I2C: Unknown operation %d\r\n", op);
                    // hub_send_tx_frame(s_unknown_cmd, sizeof(s_unknown_cmd));
                    if (cmd) { hub_sdram_free_rx((void*)cmd, rx_total); cmd = NULL; }
                    goto _continue_loop_;
                }
            }
#endif
#if IOT_HUB_GPIO
            case HUB_TARGET_ADC:{
                const hub_cfg_payload_t *pl = (const hub_cfg_payload_t *)data_addr;
                const adc_map_t *am = hub_adc_lookup(pl->pin_code);
                if (!am) {
                    DEBUG_DUMP(IOT_LOG_ERR, "HUB_TARGET_ADC: erro while doing adc map %d\r\n", op);
                    // hub_send_tx_frame(s_unknown_cmd, sizeof(s_unknown_cmd));
                    if (cmd) { hub_sdram_free_rx((void*)cmd, rx_total); cmd = NULL; }
                    goto _continue_loop_;
                }
                if (op == HUB_OP_CONFIG) {
                    if (pl->cfg == HUB_IOCT_CFG_ANALOG) {
                        HAL_ADC_DeInit(&hadc1);
                        MX_ADC1_Init();
                        ADC_ChannelConfTypeDef sConfig = {0};
                        sConfig.Channel      = am->ch;
                        sConfig.Rank         = ADC_REGULAR_RANK_1;
                        sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
                        sConfig.SingleDiff   = ADC_SINGLE_ENDED;
                        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
                            Error_Handler();
                        hub_apply_gpio_cfg(pl);
                    } else {
                        /* Change to GPIO */
                        hub_adc_to_gpio(pl->pin_code, pl);
                    }
                    if (cmd) { hub_sdram_free_rx((void*)cmd, rx_total); cmd = NULL; }
                    goto _continue_loop_;
                } else if (op == HUB_OP_READ) {
                    /* --------- READ：single sample (Polling) ---------------- */
                    ADC_ChannelConfTypeDef sConfig = {0};
                    sConfig.Channel      = am->ch;
                    sConfig.Rank         = ADC_REGULAR_RANK_1;
                    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
                    sConfig.SingleDiff   = ADC_SINGLE_ENDED;
                    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

                    HAL_ADC_Start(&hadc1);
                    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
                        /* temp reply */
                        hub_send_tx_frame(s_rsp_ok, sizeof(s_rsp_ok));
                        /* temp reply */
                        goto _continue_loop_;
                    } else {
                        hub_send_tx_frame(s_unknown_cmd, sizeof(s_unknown_cmd));
                        goto _continue_loop_;
                    }
                    HAL_ADC_Stop(&hadc1);
                } else {
                    DEBUG_DUMP(IOT_LOG_ERR, "HUB_TARGET_ADC: Unknown operation %d\r\n", op);
                    // hub_send_tx_frame(s_unknown_cmd, sizeof(s_unknown_cmd));
                    if (cmd) { hub_sdram_free_rx((void*)cmd, rx_total); cmd = NULL; }
                    goto _continue_loop_;
                }
                break;
            }
            case HUB_TARGET_PWM:{
                const pwm_map_t *pm = hub_pwm_lookup(((hub_cfg_payload_t *)data_addr)->pin_code);
                if (!pm) {
                    DEBUG_DUMP(IOT_LOG_ERR, "HUB_TARGET_PWM: erro while doing pwm map %d\r\n", op);
                    // hub_send_tx_frame(s_unknown_cmd, sizeof(s_unknown_cmd));
                    if (cmd) { hub_sdram_free_rx((void*)cmd, rx_total); cmd = NULL; }
                    goto _continue_loop_;
                }
                if (op == HUB_OP_CONFIG) {
                    const hub_cfg_payload_t *cfg = (const hub_cfg_payload_t *)data_addr;
                    if (cfg->cfg == HUB_IOCT_CFG_AF) {
                        pm->mx_reinit();
                        HAL_TIM_PWM_Start(pm->htim, pm->ch);
                    } else {
                        hub_pwm_to_gpio_generic(pm, cfg);
                    }
                    if (cmd) { hub_sdram_free_rx((void*)cmd, rx_total); cmd = NULL; }
                    goto _continue_loop_;
                } else if (op == HUB_OP_WRITE) {
                    /* payload: uint16_t duty (0~10000) */
                    uint16_t duty = *(uint16_t *)data_addr;
                    __HAL_TIM_SET_COMPARE(pm->htim, pm->ch, duty);
                    if (cmd) { hub_sdram_free_rx((void*)cmd, rx_total); cmd = NULL; }
                    goto _continue_loop_;
                } else if (op == HUB_OP_READ) {
                    /* Current duty write-back data_addr (uint16_t) */
                    uint16_t *out = (uint16_t *)data_addr;
                    *out = (uint16_t)__HAL_TIM_GET_COMPARE(pm->htim, pm->ch);
                    hub_send_tx_frame(s_rsp_ok, sizeof(s_rsp_ok));
                    goto _continue_loop_;
                } else {
                    DEBUG_DUMP(IOT_LOG_ERR, "HUB_TARGET_PWM: Unknown operation %d\r\n", op);
                    if (cmd) { hub_sdram_free_rx((void*)cmd, rx_total); cmd = NULL; }
                    goto _continue_loop_;
                }
                break;
            }
            case HUB_TARGET_DAC:{
                if (op == HUB_OP_WRITE) {
                    if (cmd) { hub_sdram_free_rx((void*)cmd, rx_total); cmd = NULL; }
                    goto _continue_loop_;
                } else {
                    DEBUG_DUMP(IOT_LOG_ERR, "HUB_TARGET_DAC: Unknown operation %d\r\n", op);
                    // hub_send_tx_frame(s_unknown_cmd, sizeof(s_unknown_cmd));
                    if (cmd) { hub_sdram_free_rx((void*)cmd, rx_total); cmd = NULL; }
                    goto _continue_loop_;
                }
                break;
            }
#endif
            case HUB_TARGET_WIFI:{
                if (op == HUB_OP_CONFIG) {
                    uart7_thread_start();
                    if (cmd) { hub_sdram_free_rx((void*)cmd, rx_total); cmd = NULL; }
                    goto _continue_loop_;
                    // tx_hdr_ptr->status = HUB_RSP_OK;
                } else if (op == HUB_OP_WRITE && in_len > 0) {
                    DEBUG_DUMP(IOT_LOG_ALL, "HUB_TARGET_UART: Sending %d bytes to UART7\r\n", in_len);
                    iot_uart7_tx_write((uint8_t *)payload, in_len);
                    if (cmd) { hub_sdram_free_rx((void*)cmd, rx_total); cmd = NULL; }
                    goto _continue_loop_;
                } else if (op == HUB_OP_READ) {
                    DEBUG_DUMP(IOT_LOG_ALL, "HUB_TARGET_UART: Receiving %d bytes from UART7\r\n", in_len);
                    (void)uart7_post_read(in_len, data_addr);
                    goto _continue_loop_;
                } else {
                    DEBUG_DUMP(IOT_LOG_ERR, "HUB_TARGET_WIFI: Unknown operation %d\r\n", op);
                    // hub_send_tx_frame(s_unknown_cmd, sizeof(s_unknown_cmd));
                    if (cmd) { hub_sdram_free_rx((void*)cmd, rx_total); cmd = NULL; }
                    goto _continue_loop_;
                }
                break;
            }
#if IOT_HUB_CAN
            case HUB_TARGET_CAN:{
                if (op == HUB_OP_WRITE) {
                    if (cmd) { hub_sdram_free_rx((void*)cmd, rx_total); cmd = NULL; }
                    goto _continue_loop_;
                } else {
                    DEBUG_DUMP(IOT_LOG_ERR, "HUB_TARGET_CAN: Unknown operation %d\r\n", op);
                    if (cmd) { hub_sdram_free_rx((void*)cmd, rx_total); cmd = NULL; }
                    goto _continue_loop_;
                }
                break;
            }
#endif
            case HUB_TARGET_SD:{
                if (op == HUB_OP_CONFIG) {
                    tx_thread_resume(&fx_app_thread);
                    if (cmd) { hub_sdram_free_rx((void*)cmd, rx_total); cmd = NULL; }
                    goto _continue_loop_;
                } else if (op == HUB_OP_READ) {
                    tx_frame_ptr = i2c_hub_filex_helper(tx_frame_ptr, cmd);
                    if (!tx_frame_ptr) {
                        hub_send_tx_frame(s_err_mem, sizeof(s_err_mem));
                        goto _continue_loop_;
                    } else {
                        /* temp reply */
                        hub_send_tx_frame(s_rsp_ok, sizeof(s_rsp_ok));
                        /* temp reply */
                        goto _continue_loop_;
                    }
                } else {
                    DEBUG_DUMP(IOT_LOG_ERR, "HUB_TARGET_CAN: Unknown operation %d\r\n", op);
                    if (cmd) { hub_sdram_free_rx((void*)cmd, rx_total); cmd = NULL; }
                    goto _continue_loop_;
                }
                break;
            }
            case HUB_TARGET_MEM:{
                if (op != HUB_OP_READ || in_len == 0U) {
                    DEBUG_DUMP(IOT_LOG_ERR, "HUB_TARGET_MEM: no accept none read operation %d\r\n", op);
                    // hub_send_tx_frame(s_unknown_cmd, sizeof(s_unknown_cmd));
                    if (cmd) { hub_sdram_free_rx((void*)cmd, rx_total); cmd = NULL; }
                    goto _continue_loop_;
                }
                uint8_t subcmd = payload[0];
                uint32_t resp_len = 0;
                switch (subcmd) {
                    case HUB_MEM_CMD_RESET:
                        hub_send_tx_flush();
                        hub_spsc_reset(&g_hub_rx_arena);
                        hub_spsc_reset(&g_hub_tx_arena);
                        if (tx_frame_ptr) {
                            uint16_t prev_payload = ((hub_rsp_t *)tx_frame_ptr)->len;
                            uint32_t prev_frame_len = (prev_payload == 0U) ?
                                                    (RSP_HDR_SZ + 4U) :
                                                    (RSP_HDR_SZ + prev_payload + 4U);
                            hub_sdram_free_tx(tx_frame_ptr, ALIGN32(prev_frame_len));
                            tx_frame_ptr = NULL;
                        }
                        resp_len = RSP_HDR_SZ + 4U;
                        tx_frame_ptr = (uint8_t *)hub_sdram_alloc_tx(ALIGN32(resp_len));
                        if (tx_frame_ptr) {
                            hub_send_tx_frame(s_rsp_ok, sizeof(s_rsp_ok));
                            if (cmd) { hub_sdram_free_rx((void*)cmd, rx_total); cmd = NULL; }
                            goto _continue_loop_;
                        } else {
                            hub_send_tx_frame(s_err_mem, sizeof(s_err_mem));
                            if (cmd) { hub_sdram_free_rx((void*)cmd, rx_total); cmd = NULL; }
                            goto _continue_loop_;
                        }
                        break;
                    case HUB_MEM_CMD_STATS:
                        if (tx_frame_ptr) {
                            uint16_t prev_payload = ((hub_rsp_t *)tx_frame_ptr)->len;
                            uint32_t prev_frame_len = (prev_payload == 0U) ?
                                                    (RSP_HDR_SZ + 4U) :
                                                    (RSP_HDR_SZ + prev_payload + 4U);
                            hub_sdram_free_tx(tx_frame_ptr, ALIGN32(prev_frame_len));
                            tx_frame_ptr = NULL;
                        }
                        uint32_t free_tx      = hub_spsc_free_space(&g_hub_tx_arena);
                        uint32_t free_rx      = hub_spsc_free_space(&g_hub_rx_arena);
                        uint32_t heap_free    = (uint32_t)hub_heap_free_bytes();
                        uint32_t heap_largest = (uint32_t)hub_heap_largest_free_block();
                        uint16_t stats_len    = 16U;
                        resp_len     = RSP_HDR_SZ + stats_len + 4U;
                        tx_frame_ptr = (uint8_t *)hub_sdram_alloc_tx(ALIGN32(resp_len));
                        if (tx_frame_ptr) {
                            hub_rsp_t *rsp = (hub_rsp_t *)tx_frame_ptr;
                            rsp->status    = HUB_RSP_OK;
                            rsp->reserved  = 0;
                            rsp->len       = stats_len;
                            rsp->data_addr = 0;
                            uint8_t *payload_dst = tx_frame_ptr + RSP_HDR_SZ;
                            memcpy(payload_dst + 0, &free_tx,      sizeof(uint32_t));
                            memcpy(payload_dst + 4, &free_rx,      sizeof(uint32_t));
                            memcpy(payload_dst + 8, &heap_free,    sizeof(uint32_t));
                            memcpy(payload_dst + 12,&heap_largest,sizeof(uint32_t));
                            uint32_t crc = iot_hub_crc32_hard(tx_frame_ptr, resp_len - 4U);
                            memcpy(tx_frame_ptr + resp_len - 4U, &crc, 4U);
                            dcache_clean32_range(tx_frame_ptr, resp_len);
                        } else {
                            hub_send_tx_frame(s_err_mem, sizeof(s_err_mem));
                            if (cmd) { hub_sdram_free_rx((void*)cmd, rx_total); cmd = NULL; }
                            goto _continue_loop_;
                        }
                        break;
                    default:{
                        DEBUG_DUMP(IOT_LOG_ERR, "HUB_TARGET_MEM: Unknown operation %d\r\n", op);
                        if (cmd) { hub_sdram_free_rx((void*)cmd, rx_total); cmd = NULL; }
                        goto _continue_loop_;
                    }
                }
                break;
            }
            default: {
                DEBUG_DUMP(IOT_LOG_ERR, "Unknown target %d\r\n", target);
                if (cmd) { hub_sdram_free_rx((void*)cmd, rx_total); cmd = NULL; }
                goto _continue_loop_;
                break;
            }
        }
        if (op != HUB_OP_READ) {
            hub_sdram_free_rx((void*)cmd, rx_total);
        }

        if (!tx_frame_ptr) {
            DEBUG_DUMP(IOT_LOG_ERR, "worker_thread_entry: tx_frame_ptr is NULL\r\n");
            goto _continue_loop_;
        }
        if (hub_send_tx_frame(tx_frame_ptr, RSP_HDR_SZ + in_len + 4U) == 0) {
            DEBUG_DUMP(IOT_LOG_ERR, "worker_thread_entry: hub_send_tx_frame failed\r\n");
            tx_frame_ptr = NULL;
        }
    _continue_loop_:
        continue;
    }
}
/* End I2C HUB worker thread */

/* crc tool */
uint32_t iot_hub_crc32_hard(const uint8_t *buf, size_t len)
{
    tx_mutex_get(&crc_mutex, TX_WAIT_FOREVER);
    __HAL_CRC_DR_RESET(&hcrc);
    uint32_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)buf, len);
    tx_mutex_put(&crc_mutex);
    return ~crc;
}