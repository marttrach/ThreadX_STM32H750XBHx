#include "i2c_hub.h"
#include "iot_util.h"
#include "iot.h"
#include <string.h>
#include <stddef.h> 
#include "usart.h"
#include "i2c.h"
#include "tx_api.h"
#include "dac.h"
#include "adc.h"
#include "crc.h"
#include "i2c_hub_uart.h"
#include "i2c_hub_modbus_server.h"
#include "i2c_hub_filex.h"

static TX_QUEUE cmd_queue;
static TX_QUEUE rsp_queue;
static TX_QUEUE i2c_tx_stage_q;
static TX_QUEUE i2c_rx_stage_q;
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

static uint8_t s_busy_frame[RSP_HDR_SZ + 4] __attribute__((aligned(32))); /* return busy frame */
static uint8_t s_worker_ack_frame[RSP_HDR_SZ + 4] __attribute__((aligned(32))); /* return worker ack frame */
// static uint8_t s_failed_frame[RSP_HDR_SZ + 4] __attribute__((aligned(32))); /* return failed frame */

/* Transacion Init*/
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
        uint32_t rx_plen = (rx_hdr_ptr->operation == HUB_OP_READ) ? 4U : ((uint32_t)rx_hdr_ptr->len + 4U);
        uint32_t rx_head_mark = hub_spsc_mark_head(&g_hub_rx_arena);
        uint32_t frame_total  = CMD_HDR_SZ + rx_plen;

        if (frame_total > HUB_I2C_RX_MAX_FRAME) {
            DEBUG_DUMP(IOT_LOG_ERR, "I2C RX: payload too large %lu > %u → BUSY\r\n",
                    (unsigned long)frame_total, (unsigned)HUB_I2C_RX_MAX_FRAME);
            dcache_clean32_range(s_busy_frame, sizeof(s_busy_frame));
            HAL_I2C_Slave_Seq_Transmit_DMA(hi2c, s_busy_frame, sizeof(s_busy_frame), I2C_LAST_FRAME);
            return;
        }

        uint8_t *rx_frame_ptr = (uint8_t*)hub_sdram_alloc_rx(frame_total);
        if (!rx_frame_ptr) {
            DEBUG_DUMP(IOT_LOG_ERR, "HAL_I2C_SlaveRxCpltCallback: hub_sdram_alloc_rx failed → BUSY\r\n");
            dcache_clean32_range(s_busy_frame, sizeof(s_busy_frame));
            HAL_I2C_Slave_Seq_Transmit_DMA(hi2c, s_busy_frame, sizeof(s_busy_frame), I2C_LAST_FRAME);
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
            HAL_I2C_EnableListen_IT(hi2c);
            return;
        }
        hub_cmd_t *cmd_ptr = (hub_cmd_t *)cur_rx.buf;
        if (tx_queue_send(&cmd_queue, &cmd_ptr, TX_NO_WAIT) != TX_SUCCESS) {
            DEBUG_DUMP(IOT_LOG_ERR, "HAL_I2C_SlaveRxCpltCallback: cmd_queue full\r\n");
            rx_txn_rollback("queue_full");
            dcache_clean32_range(s_busy_frame, sizeof(s_busy_frame));
            HAL_I2C_Slave_Seq_Transmit_DMA(hi2c, s_busy_frame, sizeof(s_busy_frame), I2C_LAST_FRAME);
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

int hub_send_tx_frame(uint8_t *buf, uint32_t total)
{
    hub_tx_task_t *task = (hub_tx_task_t*)hub_sdram_alloc_tx(ALIGN32(sizeof(hub_tx_task_t)));
    if (!task) {
        hub_sdram_free_tx(buf, ALIGN32(total));
        return 0;
    }
    dcache_clean32_range(buf, total);
    task->buf        = buf;
    task->total      = total;
    task->sent       = 0;
    task->stage_tx   = TX_STAGE_HDR;
    task->alloc_flags= 0;
    task->done_cb    = NULL;
    task->user_ctx   = NULL;

    if (tx_queue_send(&i2c_tx_stage_q, &task, TX_NO_WAIT) != TX_SUCCESS) {
        hub_sdram_free_tx(buf, ALIGN32(total));
        hub_sdram_free_tx(task, ALIGN32(sizeof(hub_tx_task_t)));
        return 0;
    }
    return 1;
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


void iot_hub_start(void)
{
    /*Init Dummy Frame */
    static ULONG cmd_queue_buf   [QUEUE_LEN        * MSG_WORDS_PTR];
    static ULONG rsp_queue_buf   [QUEUE_LEN        * MSG_WORDS_PTR];
    static ULONG tx_stage_buf    [I2C_STAGE_Q_LEN  * MSG_WORDS_PTR];
    static ULONG rx_stage_buf    [I2C_STAGE_Q_LEN  * MSG_WORDS_PTR];
    static UCHAR worker_stack[16384];
    hub_rsp_t *br_busy = (hub_rsp_t*)s_busy_frame;
    br_busy->status    = HUB_RSP_BUSY;
    br_busy->reserved  = 0;
    br_busy->len       = 0;
    br_busy->data_addr = 0;
    uint32_t crc = iot_hub_crc32_hard(s_busy_frame, RSP_HDR_SZ);
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
    // hub_rsp_t *br_failed = (hub_rsp_t*)s_failed_frame;
    // br_failed->status    = HUB_RSP_ERR_NONE;
    // br_failed->reserved  = 0;
    // br_failed->len       = 0;
    // br_failed->data_addr = 0;
    // crc = iot_hub_crc32_hard(s_failed_frame, RSP_HDR_SZ);
    // memcpy(s_failed_frame + RSP_HDR_SZ, &crc, 4);
    // dcache_clean32_range(s_failed_frame, sizeof(s_failed_frame));

    UINT s;
    s = tx_queue_create(&cmd_queue, "cmd_queue", MSG_WORDS_PTR, cmd_queue_buf, QSIZE(cmd_queue_buf));
    assert_param(s == TX_SUCCESS);
    s = tx_queue_create(&rsp_queue, "rsp_queue", MSG_WORDS_PTR, rsp_queue_buf, QSIZE(rsp_queue_buf));
    assert_param(s == TX_SUCCESS);
    s = tx_queue_create(&i2c_tx_stage_q,  "i2c_tx_stage_q",  MSG_WORDS_PTR, tx_stage_buf,  QSIZE(tx_stage_buf));
    assert_param(s == TX_SUCCESS);
    s = tx_queue_create(&i2c_rx_stage_q,  "i2c_rx_stage_q",  MSG_WORDS_PTR, rx_stage_buf,  QSIZE(rx_stage_buf));
    assert_param(s == TX_SUCCESS);
    (void)s;
    tx_mutex_create(&crc_mutex, "crc_mutex", TX_INHERIT);
    tx_thread_create(&worker_thread, "hub_worker",
                 worker_thread_entry, 0,
                 worker_stack, sizeof(worker_stack),
                 10, 10, TX_NO_TIME_SLICE, TX_DONT_START);
    hub_mem_init();
    iot_uart_init();
    HAL_I2C_EnableListen_IT(&hi2c2);
    tx_thread_resume(&worker_thread);
}

static VOID worker_thread_entry(ULONG arg)
{
    (void)arg;
    for (;;) {
        /* error work back */
        hub_err_evt_t *evt;
        uint8_t *tx_frame_ptr = NULL;
        while (tx_queue_receive(&i2c_rx_stage_q, &evt, TX_NO_WAIT) == TX_SUCCESS) {
            if (evt && evt->tag == HUB_ERR_TAG) {
                uint32_t frame_len = RSP_HDR_SZ + 4;
                uint8_t *tx_ptr = (uint8_t*)hub_sdram_alloc_tx(ALIGN32(frame_len));
                if (tx_ptr) {
                    DEBUG_DUMP(IOT_LOG_ERR, "worker_thread_entry: Sending error response\r\n");
                    hub_rsp_t *rsp = (hub_rsp_t*)tx_ptr;
                    rsp->status    = evt->status;
                    rsp->reserved  = 0;
                    rsp->len       = 0;
                    rsp->data_addr = evt->data_addr;

                    uint32_t crc = iot_hub_crc32_hard(tx_ptr, RSP_HDR_SZ);
                    memcpy(tx_ptr + RSP_HDR_SZ, &crc, 4);
                    dcache_clean32_range(tx_ptr, frame_len);

                    hub_tx_task_t *task = (hub_tx_task_t*)hub_sdram_alloc_tx(ALIGN32(sizeof(hub_tx_task_t)));
                    if (task) {
                        task->buf      = tx_ptr;
                        task->total    = frame_len;
                        task->sent     = 0;
                        task->stage_tx = TX_STAGE_HDR;
                        if (tx_queue_send(&i2c_tx_stage_q, &task, TX_NO_WAIT) != TX_SUCCESS) {
                            hub_sdram_free_tx(tx_ptr, ALIGN32(frame_len));
                            hub_sdram_free_tx(task, ALIGN32(sizeof(hub_tx_task_t)));
                        }
                    } 
                    else {
                        hub_sdram_free_tx(tx_ptr, ALIGN32(frame_len));
                    }
                }
                if (evt->rx_ptr && evt->rx_len) {
                    DEBUG_DUMP(IOT_LOG_ERR, "worker_thread_entry: Freeing rx_ptr=%p, rx_len=%ld\r\n", evt->rx_ptr, evt->rx_len);
                    hub_sdram_free_rx(evt->rx_ptr, evt->rx_len);
                }
                DEBUG_DUMP(IOT_LOG_ERR, "worker_thread_entry: Freeing evt=%p\r\n", evt);
                hub_sdram_free_tx(evt, ALIGN32(sizeof(hub_err_evt_t)));
            }
        }
        // hub_cmd_t *cmd;
        hub_cmd_t *cmd = NULL;
        tx_queue_receive(&cmd_queue, &cmd, TX_WAIT_FOREVER);
        uint8_t           target    = cmd->target;
        hub_operation_t   op        = cmd->operation;
        uint16_t          in_len    = cmd->len;
        // uint32_t          data_addr = cmd->data_addr;
        uint32_t rx_total = ALIGN32(CMD_HDR_SZ + ((op == HUB_OP_READ) ? 4U : ((uint32_t)in_len + 4U)));
        if (op == HUB_OP_READ) {
            hub_sdram_free_rx(cmd, rx_total);
            cmd = NULL;
        }
        /* CONTROLL REPLY*/
        switch (target) {
            case HUB_TARGET_UART:
                if (op == HUB_OP_CONFIG) {
                    uart8_thread_start();
                    if (cmd) { hub_sdram_free_rx((void*)cmd, rx_total); cmd = NULL; }
                    goto _continue_loop_;
                    // tx_hdr_ptr->status = HUB_RSP_OK;
                } else if (op == HUB_OP_WRITE && in_len > 0) {
                    // DEBUG_DUMP(IOT_LOG_DEBUG, "HUB_TARGET_UART: Sending %d bytes to UART8\r\n", in_len);
                    iot_uart8_tx_write((uint8_t *)cmd->payload, in_len);
                    if (cmd) { hub_sdram_free_rx((void*)cmd, rx_total); cmd = NULL; }
                    goto _continue_loop_;
                } else if (op == HUB_OP_READ) {
                    // DEBUG_DUMP(IOT_LOG_DEBUG, "HUB_TARGET_UART: Receiving %d bytes from UART8\r\n", in_len);
                    (void)uart8_post_read(in_len, cmd->data_addr);
                    goto _continue_loop_;
                } else {
                    uint32_t frame_len = RSP_HDR_SZ + 4U;
                    tx_frame_ptr = (uint8_t*)hub_sdram_alloc_tx(ALIGN32(frame_len));
                    if (tx_frame_ptr) {
                        hub_rsp_t *rsp = (hub_rsp_t*)tx_frame_ptr;
                        rsp->status    = HUB_RSP_ERR_UNKNOWN_CMD;
                        rsp->reserved  = 0;
                        rsp->len       = 0;
                        rsp->data_addr = 0;
                        uint32_t crc = iot_hub_crc32_hard(tx_frame_ptr, RSP_HDR_SZ);
                        memcpy(tx_frame_ptr + RSP_HDR_SZ, &crc, 4U);
                        dcache_clean32_range(tx_frame_ptr, frame_len);
                    }
                }
                break;
            case HUB_TARGET_GPIO:{
                switch (op) {
                    case HUB_OP_CONFIG: {
                        DEBUG_DUMP(IOT_LOG_DEBUG, "HUB_TARGET_GPIO: Configuring pin_code=0x%04X, cfg=%d, af=%d\r\n",
                                   ((hub_cfg_payload_t *)cmd->data_addr)->pin_code,
                                   ((hub_cfg_payload_t *)cmd->data_addr)->cfg,
                                   ((hub_cfg_payload_t *)cmd->data_addr)->af);
                        hub_cfg_payload_t *cfg = (hub_cfg_payload_t *)cmd->data_addr;
                        hub_apply_gpio_cfg(cfg);
                        tx_hdr_ptr->status = HUB_RSP_OK;
                        break;
                    }
                    case HUB_OP_WRITE: {
                        DEBUG_DUMP(IOT_LOG_DEBUG, "HUB_TARGET_GPIO: Writing pin_code=0x%04X, value=%d\r\n",
                                   *(uint16_t *)cmd->data_addr, *(uint8_t *)(cmd->data_addr + 2));
                        uint8_t *val = (uint8_t *)cmd->data_addr;
                        GPIO_TypeDef *port = HUB_GET_PORT(*(uint16_t *)val);
                        HAL_GPIO_WritePin(port, HUB_GET_PIN(*(uint16_t *)val), (*(val+2) ? GPIO_PIN_SET : GPIO_PIN_RESET));
                        tx_hdr_ptr->status = HUB_RSP_OK;
                        break;
                    }
                    case HUB_OP_READ: {
                        DEBUG_DUMP(IOT_LOG_DEBUG, "HUB_TARGET_GPIO: Reading pin_code=0x%04X\r\n",
                                   *(uint16_t *)cmd->data_addr);
                        uint16_t pin_code = *(uint16_t *)cmd->data_addr;
                        uint8_t *dest = (uint8_t *)cmd->data_addr;
                        GPIO_TypeDef *port = HUB_GET_PORT(pin_code);
                        dest[2] = (uint8_t)HAL_GPIO_ReadPin(port, HUB_GET_PIN(pin_code));
                        tx_hdr_ptr->len = 1;
                        tx_hdr_ptr->status = HUB_RSP_OK;
                        break;
                    }
                    default: tx_hdr_ptr->status = HUB_RSP_ERR_UNKNOWN_CMD;
                }
                break;
            }
            case HUB_TARGET_SPI: /*w5500_modbus_server_helper*/
                if (op == HUB_OP_WRITE) {
                    w5500_modbus_server_helper();
                    if (cmd) { hub_sdram_free_rx((void*)cmd, rx_total); cmd = NULL; }
                    goto _continue_loop_;
                } else {
                }
                break;
            case HUB_TARGET_I2C:
                tx_hdr_ptr->status = HUB_RSP_OK;
                break;
            case HUB_TARGET_ADC:{
                const hub_cfg_payload_t *pl = (const hub_cfg_payload_t *)cmd->data_addr;
                const adc_map_t *am = hub_adc_lookup(pl->pin_code);
                if (!am) {
                    tx_hdr_ptr->status = HUB_RSP_ERR_UNKNOWN_TARGET;
                    break;
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
                    tx_hdr_ptr->status = HUB_RSP_OK;
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
                        *(uint32_t *)cmd->data_addr = HAL_ADC_GetValue(&hadc1);
                        tx_hdr_ptr->len    = sizeof(uint32_t);
                        tx_hdr_ptr->status = HUB_RSP_OK;
                    } else {
                        tx_hdr_ptr->status = HUB_RSP_ERR_UNKNOWN_CMD;
                    }
                    HAL_ADC_Stop(&hadc1);

                } else {
                    tx_hdr_ptr->status = HUB_RSP_ERR_UNKNOWN_CMD;
                }
                break;
            }
            case HUB_TARGET_PWM:{
                const pwm_map_t *pm = hub_pwm_lookup(((hub_cfg_payload_t *)cmd->data_addr)->pin_code);
                if (!pm) {
                    tx_hdr_ptr->status = HUB_RSP_ERR_UNKNOWN_TARGET;
                    break;
                }
                if (op == HUB_OP_CONFIG) {
                    const hub_cfg_payload_t *cfg = (const hub_cfg_payload_t *)cmd->data_addr;
                    if (cfg->cfg == HUB_IOCT_CFG_AF) {
                        pm->mx_reinit();
                        HAL_TIM_PWM_Start(pm->htim, pm->ch);
                    } else {
                        hub_pwm_to_gpio_generic(pm, cfg);
                    }
                    tx_hdr_ptr->status = HUB_RSP_OK;

                } else if (op == HUB_OP_WRITE) {
                    /* payload: uint16_t duty (0~10000) */
                    uint16_t duty = *(uint16_t *)cmd->data_addr;
                    __HAL_TIM_SET_COMPARE(pm->htim, pm->ch, duty);
                    tx_hdr_ptr->status = HUB_RSP_OK;
                } else if (op == HUB_OP_READ) {
                    /* Current duty write-back data_addr (uint16_t) */
                    uint16_t *out = (uint16_t *)cmd->data_addr;
                    *out = (uint16_t)__HAL_TIM_GET_COMPARE(pm->htim, pm->ch);
                    tx_hdr_ptr->len = sizeof(uint16_t);
                    tx_hdr_ptr->status = HUB_RSP_OK;

                } else {
                    tx_hdr_ptr->status = HUB_RSP_ERR_UNKNOWN_CMD;
                }
                break;
            }
            case HUB_TARGET_DAC:
                if (op == HUB_OP_WRITE) {
                    tx_hdr_ptr->status = HUB_RSP_OK;
                } else {
                    tx_hdr_ptr->status = HUB_RSP_ERR_UNKNOWN_CMD;
                }
                break;
            case HUB_TARGET_WIFI:
                if (op == HUB_OP_CONFIG) {
                    uart7_thread_start();
                    if (cmd) { hub_sdram_free_rx((void*)cmd, rx_total); cmd = NULL; }
                    goto _continue_loop_;
                    // tx_hdr_ptr->status = HUB_RSP_OK;
                } else if (op == HUB_OP_WRITE && in_len > 0) {
                    // DEBUG_DUMP(IOT_LOG_DEBUG, "HUB_TARGET_UART: Sending %d bytes to UART7\r\n", in_len);
                    iot_uart7_tx_write((uint8_t *)cmd->payload, in_len);
                    if (cmd) { hub_sdram_free_rx((void*)cmd, rx_total); cmd = NULL; }
                    goto _continue_loop_;
                } else if (op == HUB_OP_READ) {
                    // DEBUG_DUMP(IOT_LOG_DEBUG, "HUB_TARGET_UART: Receiving %d bytes from UART7\r\n", in_len);
                    (void)uart7_post_read(in_len, cmd->data_addr);
                    goto _continue_loop_;
                } else {
                    uint32_t frame_len = RSP_HDR_SZ + 4U;
                    tx_frame_ptr = (uint8_t*)hub_sdram_alloc_tx(ALIGN32(frame_len));
                    if (tx_frame_ptr) {
                        hub_rsp_t *rsp = (hub_rsp_t*)tx_frame_ptr;
                        rsp->status    = HUB_RSP_ERR_UNKNOWN_CMD;
                        rsp->reserved  = 0;
                        rsp->len       = 0;
                        rsp->data_addr = 0;
                        uint32_t crc = iot_hub_crc32_hard(tx_frame_ptr, RSP_HDR_SZ);
                        memcpy(tx_frame_ptr + RSP_HDR_SZ, &crc, 4U);
                        dcache_clean32_range(tx_frame_ptr, frame_len);
                    }
                }
                break;
            case HUB_TARGET_CAN:
                if (op == HUB_OP_WRITE && in_len > 0) {
                    tx_hdr_ptr->status = HUB_RSP_OK;
                } else {
                    tx_hdr_ptr->status = HUB_RSP_ERR_UNKNOWN_CMD;
                }
                break;
            case HUB_TARGET_SD:
                if (op == HUB_OP_READ) {
                    tx_frame_ptr = i2c_hub_filex_helper(tx_frame_ptr, cmd);
                    if (!tx_frame_ptr) {
                        tx_hdr_ptr->status = HUB_RSP_ERR_MEMORY;
                    } else {
                        tx_hdr_ptr->status = HUB_RSP_OK;
                    }
                } else {
                    tx_hdr_ptr->status = HUB_RSP_ERR_UNKNOWN_CMD;
                }
                break;
            case HUB_TARGET_MEM:{
                if (op != HUB_OP_READ || in_len == 0U) {
                    tx_hdr_ptr->status = HUB_RSP_ERR_UNKNOWN_CMD;
                    break;
                }
                uint8_t subcmd = cmd->payload[0];
                uint32_t resp_len = 0;
                switch (subcmd) {
                    case HUB_MEM_CMD_RESET:
                        iot_hub_tx_flush();
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
                            hub_rsp_t *rsp = (hub_rsp_t *)tx_frame_ptr;
                            rsp->status    = HUB_RSP_OK;
                            rsp->reserved  = 0;
                            rsp->len       = 0;
                            rsp->data_addr = 0;
                            uint32_t crc = iot_hub_crc32_hard(tx_frame_ptr, RSP_HDR_SZ);
                            memcpy(tx_frame_ptr + RSP_HDR_SZ, &crc, 4U);
                            dcache_clean32_range(tx_frame_ptr, resp_len);
                        } else {
                            tx_hdr_ptr->status = HUB_RSP_ERR_MEMORY;
                            tx_hdr_ptr->len    = 0;
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
                            tx_hdr_ptr->status = HUB_RSP_ERR_MEMORY;
                            tx_hdr_ptr->len    = 0;
                        }
                        break;
                    default:
                        tx_hdr_ptr->status = HUB_RSP_ERR_UNKNOWN_CMD;
                        tx_hdr_ptr->len    = 0;
                        break;
                }
                break;
            }
            default: {
                uint32_t frame_len = RSP_HDR_SZ + 4U;
                tx_frame_ptr = (uint8_t*)hub_sdram_alloc_tx(ALIGN32(frame_len));
                if (tx_frame_ptr) {
                    hub_rsp_t *rsp = (hub_rsp_t*)tx_frame_ptr;
                    rsp->status    = HUB_RSP_ERR_UNKNOWN_TARGET;
                    rsp->reserved  = 0;
                    rsp->len       = 0;
                    rsp->data_addr = 0;
                    uint32_t crc = iot_hub_crc32_hard(tx_frame_ptr, RSP_HDR_SZ);
                    memcpy(tx_frame_ptr + RSP_HDR_SZ, &crc, 4U);
                    dcache_clean32_range(tx_frame_ptr, frame_len);
                }
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
        hub_tx_task_t *task = (hub_tx_task_t*)hub_sdram_alloc_tx(ALIGN32(sizeof(hub_tx_task_t)));
        if (!task) {
            hub_sdram_free_tx(tx_frame_ptr, ALIGN32(RSP_HDR_SZ + ((hub_rsp_t*)tx_frame_ptr)->len + 4U));
            tx_frame_ptr = NULL;
            continue;
        }
        uint32_t frame_len = RSP_HDR_SZ + in_len + 4;
        dcache_clean32_range(tx_frame_ptr, frame_len);
        task->buf      = tx_frame_ptr;
        task->total    = frame_len;
        task->sent     = 0U;
        task->stage_tx = TX_STAGE_HDR;
        if (tx_queue_send(&i2c_tx_stage_q, &task, TX_NO_WAIT) != TX_SUCCESS) {
            hub_sdram_free_tx(tx_frame_ptr, ALIGN32(frame_len));
            hub_sdram_free_tx(task, ALIGN32(sizeof(hub_tx_task_t)));
        } else {
            tx_frame_ptr = NULL;
        }
    _continue_loop_:
        continue;
    }
}

void iot_hub_tx_flush(void)
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


uint32_t iot_hub_crc32_hard(const uint8_t *buf, size_t len)
{
    tx_mutex_get(&crc_mutex, TX_WAIT_FOREVER);
    __HAL_CRC_DR_RESET(&hcrc);
    uint32_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)buf, len);
    tx_mutex_put(&crc_mutex);
    return ~crc;
}
