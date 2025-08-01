#include "i2c_hub.h"
#include "iot_util.h"
#include "iot.h"
#include <string.h>
#include "usart.h"
#include "i2c.h"
#include "tx_api.h"
#include "dac.h"
#include "adc.h"
#include "crc.h"
#include "i2c_hub_mdma.h"
#include "mdma.h"
#include <stddef.h> 
#include "i2c_hub_uart.h"

static TX_QUEUE cmd_queue;
static TX_QUEUE rsp_queue;
static TX_MUTEX crc_mutex; 
static TX_THREAD worker_thread;

static VOID worker_thread_entry(ULONG arg);

// static hub_cmd_t cmd_in  __attribute__((aligned(32)));
// static hub_rsp_t rsp_out __attribute__((aligned(32)));
static uint32_t*   hub_mem_base = (uint32_t*)HUB_SDRAM_BASE;

static uint8_t  rx_hdr[CMD_HDR_SZ]  __attribute__((aligned(32)));
static hub_cmd_t *rx_hdr_ptr = (hub_cmd_t *)rx_hdr;
static uint8_t *rx_frame_ptr = NULL;   // header + payload + CRC
static uint16_t rx_plen      = 0;
static enum { RX_STAGE_HDR, RX_STAGE_PAY } rx_stage = RX_STAGE_HDR;

static uint8_t  tx_hdr[RSP_HDR_SZ]  __attribute__((aligned(32)));
static hub_rsp_t *tx_hdr_ptr = (hub_rsp_t *)tx_hdr;
static uint8_t *tx_frame_ptr = NULL;   // header + payload + CRC
static uint16_t tx_plen      = 0;

static uint32_t sdram_ofs = 0;
static hub_tx_ctx_t g_tx = {0};

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t dir, uint16_t addr7)
{
    if (hi2c != &hi2c2 || addr7 != HUB_I2C_ADDR) return;
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);
    if (dir == I2C_DIRECTION_TRANSMIT)      /* Master to Slave */
    {
        SCB_CleanDCache_by_Addr((uint32_t *)&rx_hdr, ALIGN32(CMD_HDR_SZ));
        HAL_I2C_Slave_Seq_Receive_DMA(hi2c, rx_hdr, CMD_HDR_SZ, I2C_LAST_FRAME);
    }else if (dir == I2C_DIRECTION_RECEIVE) {
        /* only tx when having msg in rsp queue */
        if (tx_queue_receive(&rsp_queue, &g_tx.buf, TX_NO_WAIT) == TX_SUCCESS) {
            g_tx.total = (g_tx.stage == TX_STAGE_PAY) ? RSP_HDR_SZ + ((hub_rsp_t*)g_tx.buf)->len + 4 : RSP_HDR_SZ + 4;
            // g_tx.total = RSP_HDR_SZ + ((hub_rsp_t*)g_tx.buf)->len + 4;   /* +CRC  */
            g_tx.sent  = 0;
            uint16_t first = RSP_HDR_SZ;
            SCB_CleanDCache_by_Addr((uint32_t*)g_tx.buf, ALIGN32(first)); 
            HAL_I2C_Slave_Seq_Transmit_DMA(hi2c, g_tx.buf, first, I2C_FIRST_FRAME);
            g_tx.sent = first;
        } 
        else{
            return;
        }
    }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c != &hi2c2) return;
    if (rx_stage == RX_STAGE_HDR){
        /* Header */
        SCB_InvalidateDCache_by_Addr((uint32_t *)rx_hdr, ALIGN32(CMD_HDR_SZ));
        rx_plen = rx_hdr_ptr->len + 4;; // +4 for CRC
        if(rx_plen > HUB_SDRAM_SIZE - sdram_ofs) {
            DEBUG_DUMP(IOT_LOG_DEBUG, "i2c_hub: payload too large: %d > %ld\r\n", rx_plen, HUB_SDRAM_SIZE - sdram_ofs);
            hub_rsp_t bad_header = {
                .status = HUB_RSP_ERR_UNKNOWN_CMD,
                .len    = 0,
                .data_addr = rx_hdr_ptr->data_addr
            };
            tx_frame_ptr = hub_sdram_alloc(&sdram_ofs, ALIGN32(RSP_HDR_SZ + bad_header.len + 4));
            memcpy(tx_frame_ptr, tx_hdr, RSP_HDR_SZ);
            uint8_t *payload_dst = tx_frame_ptr + RSP_HDR_SZ;
            uint32_t rsp_crc_calc = iot_hub_crc32_hard(tx_frame_ptr, RSP_HDR_SZ);
            memcpy(payload_dst, &rsp_crc_calc, 4);
            tx_queue_send(&rsp_queue, &tx_frame_ptr, TX_NO_WAIT);
            HAL_I2C_EnableListen_IT(hi2c);
            return;
        }
        rx_frame_ptr = hub_sdram_alloc(&sdram_ofs, ALIGN32(CMD_HDR_SZ + rx_plen));
        memcpy(rx_frame_ptr, rx_hdr, CMD_HDR_SZ);

        uint8_t *payload_dst = rx_frame_ptr + CMD_HDR_SZ;
        SCB_CleanDCache_by_Addr((uint32_t *)payload_dst, ALIGN32(rx_plen));
        rx_stage = RX_STAGE_PAY;
        DEBUG_DUMP(IOT_LOG_DEBUG, "GET I2C~\r\n");
        HAL_I2C_Slave_Seq_Receive_DMA(hi2c, payload_dst, rx_plen, I2C_LAST_FRAME);
        return; 
    }
    if (rx_stage == RX_STAGE_PAY) {
        /* payload+CRC */
        DEBUG_DUMP(IOT_LOG_DEBUG, "Error ??\r\n");
        uint8_t *payload_ptr = rx_frame_ptr + CMD_HDR_SZ;
        SCB_InvalidateDCache_by_Addr((uint32_t *)payload_ptr, ALIGN32(rx_plen));
        uint32_t crc_rx;
        DEBUG_DUMP(IOT_LOG_DEBUG, "Error 1\r\n");
        memcpy(&crc_rx, payload_ptr + rx_plen - 4, 4);
        DEBUG_DUMP(IOT_LOG_DEBUG, "Error 2\r\n");
        uint32_t cmd_crc_calc = iot_hub_crc32_hard(rx_frame_ptr, CMD_HDR_SZ + rx_plen - 4);
        if (cmd_crc_calc != crc_rx) {
            hub_rsp_t bad_header = {
                .status = HUB_RSP_ERR_CRC,
                .len    = 0,
                .data_addr = rx_hdr_ptr->data_addr
            };
            DEBUG_DUMP(IOT_LOG_DEBUG, "Error 3\r\n");
            tx_frame_ptr = hub_sdram_alloc(&sdram_ofs, ALIGN32(RSP_HDR_SZ + bad_header.len + 4));
            DEBUG_DUMP(IOT_LOG_DEBUG, "Error 4\r\n");
            memcpy(tx_frame_ptr, tx_hdr, RSP_HDR_SZ);
            DEBUG_DUMP(IOT_LOG_DEBUG, "Error 5\r\n");
            uint8_t *payload_dst = tx_frame_ptr + RSP_HDR_SZ;
            uint32_t rsp_crc_calc = iot_hub_crc32_hard(tx_frame_ptr, RSP_HDR_SZ);
            memcpy(payload_dst, &rsp_crc_calc, 4);
            DEBUG_DUMP(IOT_LOG_DEBUG, "Error 6\r\n");
            tx_queue_send(&rsp_queue, &tx_frame_ptr, TX_NO_WAIT);
            DEBUG_DUMP(IOT_LOG_DEBUG, "Error 7\r\n");
            HAL_I2C_EnableListen_IT(hi2c);
            return;
        } else {
            DEBUG_DUMP(IOT_LOG_DEBUG, "success 1\r\n");
            tx_queue_send(&cmd_queue, &rx_frame_ptr, TX_NO_WAIT);
            DEBUG_DUMP(IOT_LOG_DEBUG, "success 2\r\n");

        }
        DEBUG_DUMP(IOT_LOG_DEBUG, "PAY I2C~\r\n");
        rx_stage = RX_STAGE_HDR;
        HAL_I2C_EnableListen_IT(hi2c);
    }
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c != &hi2c2) return;
    if (g_tx.sent < g_tx.total) {
        uint16_t remain = g_tx.total - g_tx.sent;
        DEBUG_DUMP(IOT_LOG_DEBUG, "need trans remain len: %d\r\n", remain);
        uint8_t *next   = g_tx.buf + g_tx.sent;
        SCB_CleanDCache_by_Addr((uint32_t*)next, ALIGN32(remain)); 
        g_tx.sent += remain;
        HAL_I2C_Slave_Seq_Transmit_DMA(hi2c, next, remain, I2C_LAST_FRAME);
        return;
    }
    g_tx.buf = NULL;
    HAL_I2C_EnableListen_IT(hi2c);
}
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c != &hi2c2) return;
    HAL_I2C_EnableListen_IT(hi2c);
}
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c != &hi2c2) return;
    __HAL_I2C_CLEAR_FLAG(hi2c,
        I2C_FLAG_BERR | I2C_FLAG_ARLO | I2C_FLAG_AF);
    HAL_I2C_EnableListen_IT(hi2c);
}

void iot_hub_start(void)
{
    /*Init Dummy Frame */
    static UCHAR cmd_queue_buf[QUEUE_LEN * sizeof(hub_cmd_t)];
    static UCHAR rsp_queue_buf[QUEUE_LEN * sizeof(hub_rsp_t)];
    static UCHAR worker_stack[2048];
    tx_queue_create(&cmd_queue, "cmd_queue", MSG_WORDS_CMD, cmd_queue_buf, sizeof(cmd_queue_buf));
    tx_queue_create(&rsp_queue, "rsp_queue", MSG_WORDS_RSP, rsp_queue_buf, sizeof(rsp_queue_buf));
    tx_mutex_create(&crc_mutex, "crc_mutex", TX_INHERIT);
    tx_thread_create(&worker_thread, "hub_worker",
                 worker_thread_entry, 0,
                 worker_stack, sizeof(worker_stack),
                 10, 10, TX_NO_TIME_SLICE, TX_DONT_START);
    HAL_I2C_EnableListen_IT(&hi2c2);
    iot_uart_init();
    tx_thread_resume(&worker_thread);
}

static VOID worker_thread_entry(ULONG arg)
{
    (void)arg;
    for (;;) {
        hub_cmd_t *cmd;
        tx_queue_receive(&cmd_queue, &cmd, TX_WAIT_FOREVER);
        DEBUG_DUMP(IOT_LOG_DEBUG, "success 3\r\n");
        if (cmd->data_addr == 0){
            cmd->data_addr = (uint32_t)hub_mem_base + (uint32_t)cmd->data_addr;
        }
        /* CONTROLL REPLY*/
        switch (cmd->target) {
            case HUB_TARGET_UART:
                DEBUG_DUMP(IOT_LOG_DEBUG, "success 4\r\n");
                if (cmd->operation == HUB_OP_WRITE && cmd->len > 0) {
                    DEBUG_DUMP(IOT_LOG_DEBUG, "HUB_TARGET_UART: Transmitting %d bytes\r\n", cmd->len);
                    iot_uart8_tx_write((uint8_t *)cmd->payload, cmd->len);
                } else if(cmd->operation == HUB_OP_READ) {
                    uint32_t available = uart8_rx_available();
                    if (cmd->len == 0) { /* PEEK */
                        g_tx.stage = TX_STAGE_HDR;
                        tx_hdr_ptr->len = (uint16_t)available;
                        DEBUG_DUMP(IOT_LOG_DEBUG, "HUB_TARGET_UART: PEEK %d bytes from UART8\r\n", tx_hdr_ptr->len);
                        tx_hdr_ptr->status = HUB_RSP_OK;
                        tx_hdr_ptr->data_addr = cmd->data_addr;
                        tx_hdr_ptr->reserved = 0;
                    } else{
                        g_tx.stage = TX_STAGE_PAY;
                        uint16_t need = MIN(cmd->len, available);
                        uart8_rx_read(cmd->payload, need);
                        tx_hdr_ptr->len = (uint16_t)need;
                        DEBUG_DUMP(IOT_LOG_DEBUG, "HUB_TARGET_UART: Receiving %d bytes from UART8\r\n", need);
                        tx_hdr_ptr->data_addr = cmd->data_addr;
                        tx_hdr_ptr->status = HUB_RSP_OK;
                    }
                } else {
                    tx_hdr_ptr->status = HUB_RSP_ERR_UNKNOWN_CMD;
                }
                break;
            case HUB_TARGET_GPIO:{
                switch (cmd->operation) {
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
            case HUB_TARGET_SPI:
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
                if (cmd->operation == HUB_OP_CONFIG) {
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
                } else if (cmd->operation == HUB_OP_READ) {
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
                if (cmd->operation == HUB_OP_CONFIG) {
                    const hub_cfg_payload_t *cfg = (const hub_cfg_payload_t *)cmd->data_addr;
                    if (cfg->cfg == HUB_IOCT_CFG_AF) {
                        pm->mx_reinit();
                        HAL_TIM_PWM_Start(pm->htim, pm->ch);
                    } else {
                        hub_pwm_to_gpio_generic(pm, cfg);
                    }
                    tx_hdr_ptr->status = HUB_RSP_OK;

                } else if (cmd->operation == HUB_OP_WRITE) {
                    /* payload: uint16_t duty (0~10000) */
                    uint16_t duty = *(uint16_t *)cmd->data_addr;
                    __HAL_TIM_SET_COMPARE(pm->htim, pm->ch, duty);
                    tx_hdr_ptr->status = HUB_RSP_OK;
                } else if (cmd->operation == HUB_OP_READ) {
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
                if (cmd->operation == HUB_OP_WRITE) {
                    tx_hdr_ptr->status = HUB_RSP_OK;
                } else {
                    tx_hdr_ptr->status = HUB_RSP_ERR_UNKNOWN_CMD;
                }
                break;
            case HUB_TARGET_WIFI:
                if (cmd->operation == HUB_OP_WRITE && cmd->len > 0) {                    
                    DEBUG_DUMP(IOT_LOG_DEBUG, "HUB_TARGET_WIFI: Transmitting %d bytes\r\n", cmd->len);
                    HAL_UART_Transmit_DMA(&huart7, (uint8_t *)cmd->payload, cmd->len);
                } else if(cmd->operation == HUB_OP_READ) {
                    if (cmd->len == 0) { /* PEEK */
                        uint32_t n = uart7_rx_available();
                        tx_hdr_ptr->len = n;
                        DEBUG_DUMP(IOT_LOG_DEBUG, "HUB_TARGET_WIFI: PEEK %d bytes from UART7\r\n", tx_hdr_ptr->len);
                        tx_hdr_ptr->status = HUB_RSP_OK;
                        tx_hdr_ptr->data_addr = cmd->data_addr;
                        HAL_UART_Receive_DMA(&huart7, (uint8_t *)cmd->payload, cmd->len);
                    }else{
                        DEBUG_DUMP(IOT_LOG_DEBUG, "HUB_TARGET_UART: Receiving %d bytes from UART7\r\n", cmd->len);
                        // HAL_UART_Receive_DMA(&huart7, (uint8_t *)cmd->payload, cmd->len);
                        iot_uart7_tx_write((uint8_t *)cmd->payload, cmd->len);
                        tx_hdr_ptr->len = cmd->len;
                        tx_hdr_ptr->data_addr = cmd->data_addr;
                        tx_hdr_ptr->status = HUB_RSP_OK;
                    }
                } else {
                    tx_hdr_ptr->status = HUB_RSP_ERR_UNKNOWN_CMD;
                }
                break;
            case HUB_TARGET_CAN:
                if (cmd->operation == HUB_OP_WRITE && cmd->len > 0) {
                    tx_hdr_ptr->status = HUB_RSP_OK;
                } else {
                    tx_hdr_ptr->status = HUB_RSP_ERR_UNKNOWN_CMD;
                }
                break;
            case HUB_TARGET_SD:
                if (cmd->operation == HUB_OP_READ || cmd->operation == HUB_OP_WRITE) {
                    // Implement SD read/write logic here
                    // This is a placeholder for actual SD read/write logic
                    tx_hdr_ptr->status = HUB_RSP_OK;
                } else {
                    tx_hdr_ptr->status = HUB_RSP_ERR_UNKNOWN_CMD;
                }
                break;
            default:
                tx_hdr_ptr->status = HUB_RSP_ERR_UNKNOWN_TARGET;
                break;
        }
        if (cmd->operation == HUB_OP_WRITE || cmd->operation == HUB_OP_CONFIG){
            continue;
        }
        tx_plen = (g_tx.stage == TX_STAGE_PAY) ? tx_hdr_ptr->len : 0;
        if (tx_hdr_ptr->data_addr == 0) {
            tx_hdr_ptr->data_addr = cmd->data_addr;
        }
        tx_frame_ptr = hub_sdram_alloc(&sdram_ofs, ALIGN32(RSP_HDR_SZ + tx_plen + 4));
        // DEBUG_DUMP(IOT_LOG_DEBUG, "tx_frame_ptr malloc: ");
        // for(uint8_t i = 0; i < RSP_HDR_SZ + tx_plen + 4; i++){
        //     DEBUG_DUMP(IOT_LOG_DEBUG, "%02x ", tx_frame_ptr[i]);
        // }
        // DEBUG_DUMP(IOT_LOG_DEBUG, "\r\n");
        SCB_CleanDCache_by_Addr((uint32_t*)tx_frame_ptr, ALIGN32(RSP_HDR_SZ + tx_plen + 4));
        
        // DEBUG_DUMP(IOT_LOG_DEBUG, "status: %d\r\n", tx_hdr_ptr->status);
        // DEBUG_DUMP(IOT_LOG_DEBUG, "reserved: %d\r\n", tx_hdr_ptr->reserved);
        // DEBUG_DUMP(IOT_LOG_DEBUG, "len: %d\r\n", tx_hdr_ptr->len);
        // DEBUG_DUMP(IOT_LOG_DEBUG, "data_addr: %lx\r\n", tx_hdr_ptr->data_addr);

        memcpy(tx_frame_ptr, tx_hdr, RSP_HDR_SZ);
        
        // DEBUG_DUMP(IOT_LOG_DEBUG, "tx_frame_ptr after copy: ");
        // for(uint8_t i = 0; i < RSP_HDR_SZ + tx_plen + 4; i++){
        //     DEBUG_DUMP(IOT_LOG_DEBUG, "%02x ", tx_frame_ptr[i]);
        // }
        // DEBUG_DUMP(IOT_LOG_DEBUG, "\r\n");
        /*first header  no copy*/
        if (tx_plen > 0 && g_tx.stage == TX_STAGE_PAY) {
            memcpy(tx_frame_ptr + RSP_HDR_SZ, (uint8_t *)tx_hdr_ptr->data_addr, tx_plen);
        }
        uint32_t rsp_crc_calc = iot_hub_crc32_hard(tx_frame_ptr, RSP_HDR_SZ + tx_plen);
        DEBUG_DUMP(IOT_LOG_DEBUG, "rsp_crc_calc: %ld \r\n", rsp_crc_calc);
        memcpy(tx_frame_ptr + RSP_HDR_SZ + tx_plen, &rsp_crc_calc, sizeof(rsp_crc_calc));
        SCB_CleanDCache_by_Addr((uint32_t*)tx_frame_ptr, ALIGN32(RSP_HDR_SZ + tx_plen + 4));
        DEBUG_DUMP(IOT_LOG_DEBUG, "rsp + crc: ");
        for(uint8_t i = 0; i < RSP_HDR_SZ + tx_plen + 4; i++){
            DEBUG_DUMP(IOT_LOG_DEBUG, "%02x ", tx_frame_ptr[i]);
        }
        DEBUG_DUMP(IOT_LOG_DEBUG, "\r\n");
        DEBUG_DUMP(IOT_LOG_DEBUG, "copy success\r\n");
        DEBUG_DUMP(IOT_LOG_DEBUG, "tx_hdr_ptr status: %d\r\n", tx_hdr_ptr->status);
        DEBUG_DUMP(IOT_LOG_DEBUG, "tx_hdr_ptr reserved: %d\r\n", tx_hdr_ptr->reserved);
        DEBUG_DUMP(IOT_LOG_DEBUG, "tx_hdr_ptr len: %d\r\n", tx_hdr_ptr->len);
        DEBUG_DUMP(IOT_LOG_DEBUG, "tx_hdr_ptr data_addr: 0x%08lx\r\n", tx_hdr_ptr->data_addr);
        DEBUG_DUMP(IOT_LOG_DEBUG, "tx_hdr_ptr crc: 0x%08lx\r\n", &tx_hdr_ptr->payload);
        tx_queue_send(&rsp_queue, &tx_frame_ptr, TX_NO_WAIT);
        tx_plen = 0;
    }
}

uint32_t iot_hub_crc32_hard(const uint8_t *buf, size_t len)
{
    tx_mutex_get(&crc_mutex, TX_WAIT_FOREVER);
    __HAL_CRC_DR_RESET(&hcrc);
    uint32_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)buf, len);
    tx_mutex_put(&crc_mutex);
    return ~crc;
}
