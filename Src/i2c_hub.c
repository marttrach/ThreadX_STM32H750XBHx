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
#include "app_filex.h"

static TX_QUEUE cmd_queue;
static TX_QUEUE rsp_queue;
static TX_MUTEX crc_mutex; 
static TX_THREAD worker_thread;

static VOID worker_thread_entry(ULONG arg);

static hub_cmd_t cmd_in  __attribute__((aligned(32)));
static hub_rsp_t rsp_out __attribute__((aligned(32)));
static uint32_t*   hub_mem_base = (uint32_t*)HUB_SDRAM_BASE;

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t dir, uint16_t addr7)
{
    if (hi2c != &hi2c2 || addr7 != HUB_I2C_ADDR) return;
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);

    if (dir == I2C_DIRECTION_TRANSMIT){
        SCB_CleanDCache_by_Addr((uint32_t *)&cmd_in, ALIGN32(sizeof(cmd_in)));
        HAL_I2C_Slave_Seq_Receive_DMA(hi2c, (uint8_t*)&cmd_in, sizeof(cmd_in), I2C_LAST_FRAME);
    }else{
        if (tx_queue_receive(&rsp_queue, &rsp_out, TX_NO_WAIT) != TX_SUCCESS)memset(&rsp_out, 0xFF, sizeof(rsp_out));
        HAL_I2C_Slave_Seq_Transmit_DMA(hi2c, (uint8_t*)&rsp_out, sizeof(rsp_out), I2C_LAST_FRAME);
    }
}
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c != &hi2c2) return;
    SCB_InvalidateDCache_by_Addr((uint32_t *)&cmd_in, ALIGN32(sizeof(cmd_in)));
    tx_queue_send(&cmd_queue, &cmd_in, TX_NO_WAIT);
    HAL_I2C_EnableListen_IT(hi2c);
}
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c != &hi2c2) return;
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
    tx_thread_resume(&worker_thread);
    // tx_thread_resume(&fx_app_thread);
}

static uint32_t iot_hub_crc32(const uint8_t *buf, size_t len)
{
    // uint32_t words = (len + 3) >> 2;

    tx_mutex_get(&crc_mutex, TX_WAIT_FOREVER);
    __HAL_CRC_DR_RESET(&hcrc);
    uint32_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)buf, len);
    tx_mutex_put(&crc_mutex);

    return ~crc;
}

static VOID worker_thread_entry(ULONG arg)
{
    (void)arg;
    for (;;) {
        hub_cmd_t cmd;
        tx_queue_receive(&cmd_queue, &cmd, TX_WAIT_FOREVER);
        DEBUG_DUMP(IOT_LOG_DEBUG, "CMD: target=%d, op=%d, len=%d, addr=0x%08lX, crc=0x%08lX\r\n",
                   cmd.target, cmd.operation, cmd.len, cmd.data_addr, cmd.crc);
        if (cmd.data_addr == 0){
            cmd.data_addr = (uint32_t)hub_mem_base + (uint32_t)cmd.data_addr;
        }
        /* CRC */
        uint32_t crc_calc = iot_hub_crc32((uint8_t*)&cmd, sizeof(cmd) - sizeof(cmd.crc));
        /* Error CRC Reply*/
        if (crc_calc != cmd.crc) {
            hub_rsp_t bad = {
                .status    = HUB_RSP_ERR_CRC,
                .reserved  = 0,
                .len       = 0,
                .data_addr = cmd.data_addr
            };
            bad.crc = iot_hub_crc32((uint8_t*)&bad, sizeof(bad) - sizeof(bad.crc));
            DEBUG_DUMP(IOT_LOG_DEBUG, "CMD CRC ERROR: 0x%08lX != 0x%08lX\r\n", crc_calc, cmd.crc);
            tx_queue_send(&rsp_queue, &bad, TX_NO_WAIT);
            continue; /* Error Handler ?*/
        }
        DEBUG_DUMP(IOT_LOG_DEBUG, "CMD CRC OK: 0x%08lX\r\n", crc_calc);
        /* CONTROLL REPLY*/
        hub_rsp_t rsp = {0};
        rsp.data_addr = cmd.data_addr;
        rsp.len = 0;
        rsp.crc = iot_hub_crc32((uint8_t*)&rsp, sizeof(rsp) - sizeof(rsp.crc));
        DEBUG_DUMP(IOT_LOG_DEBUG, "RSP: status=%d, len=%d, addr=0x%08lX, crc=0x%08lX\r\n",
                   rsp.status, rsp.len, rsp.data_addr, rsp.crc);
        switch (cmd.target) {
            case HUB_TARGET_UART:
                DEBUG_DUMP(IOT_LOG_DEBUG, "HUB_TARGET_UART: op=%d, len=%d, addr=0x%08lX\r\n",
                           cmd.operation, cmd.len, cmd.data_addr);
                if (cmd.operation == HUB_OP_WRITE && cmd.len > 0) {
                    DEBUG_DUMP(IOT_LOG_DEBUG, "HUB_TARGET_UART: Transmitting %d bytes\r\n", cmd.len);
                    HAL_UART_Transmit_DMA(&huart8, (uint8_t*)cmd.data_addr, cmd.len);
                    rsp.status = HUB_RSP_OK;
                } else if(cmd.operation == HUB_OP_READ) {
                    DEBUG_DUMP(IOT_LOG_DEBUG, "HUB_TARGET_UART: Receiving %d bytes\r\n", cmd.len);
                    HAL_UART_Receive_DMA(&huart8, (uint8_t*)cmd.data_addr, cmd.len);
                    rsp.len = cmd.len;
                    rsp.data_addr = cmd.data_addr;
                    rsp.status = HUB_RSP_OK;
                } else {
                    rsp.status = HUB_RSP_ERR_UNKNOWN_CMD;
                }
                break;
            case HUB_TARGET_GPIO:{
                switch (cmd.operation) {
                    case HUB_OP_CONFIG: {
                        hub_cfg_payload_t *cfg = (hub_cfg_payload_t *)cmd.data_addr;
                        hub_apply_gpio_cfg(cfg);
                        rsp.status = HUB_RSP_OK;
                        break;
                    }
                    case HUB_OP_WRITE: {
                        uint8_t *val = (uint8_t *)cmd.data_addr;
                        GPIO_TypeDef *port = HUB_GET_PORT(*(uint16_t *)val);
                        HAL_GPIO_WritePin(port, HUB_GET_PIN(*(uint16_t *)val), (*(val+2) ? GPIO_PIN_SET : GPIO_PIN_RESET));
                        rsp.status = HUB_RSP_OK;
                        break;
                    }
                    case HUB_OP_READ: {
                        uint16_t pin_code = *(uint16_t *)cmd.data_addr;
                        uint8_t *dest = (uint8_t *)cmd.data_addr;
                        GPIO_TypeDef *port = HUB_GET_PORT(pin_code);
                        dest[2] = (uint8_t)HAL_GPIO_ReadPin(port, HUB_GET_PIN(pin_code));
                        rsp.len = 1;
                        rsp.status = HUB_RSP_OK;
                        break;
                    }
                    default: rsp.status = HUB_RSP_ERR_UNKNOWN_CMD;
                }
                break;
            }
            case HUB_TARGET_SPI:
            case HUB_TARGET_I2C:
                rsp.status = HUB_RSP_OK;
                break;
            case HUB_TARGET_ADC:{
                const hub_cfg_payload_t *pl = (const hub_cfg_payload_t *)cmd.data_addr;
                const adc_map_t *am = hub_adc_lookup(pl->pin_code);
                if (!am) {
                    rsp.status = HUB_RSP_ERR_UNKNOWN_TARGET;
                    break;
                }
                if (cmd.operation == HUB_OP_CONFIG) {
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
                    rsp.status = HUB_RSP_OK;
                } else if (cmd.operation == HUB_OP_READ) {
                    /* --------- READ：single sample (Polling) ---------------- */
                    ADC_ChannelConfTypeDef sConfig = {0};
                    sConfig.Channel      = am->ch;
                    sConfig.Rank         = ADC_REGULAR_RANK_1;
                    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
                    sConfig.SingleDiff   = ADC_SINGLE_ENDED;
                    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

                    HAL_ADC_Start(&hadc1);
                    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
                        *(uint32_t *)cmd.data_addr = HAL_ADC_GetValue(&hadc1);
                        rsp.len    = sizeof(uint32_t);
                        rsp.status = HUB_RSP_OK;
                    } else {
                        rsp.status = HUB_RSP_ERR_UNKNOWN_CMD;
                    }
                    HAL_ADC_Stop(&hadc1);

                } else {
                    rsp.status = HUB_RSP_ERR_UNKNOWN_CMD;
                }
                break;
            }
            case HUB_TARGET_PWM:{
                const pwm_map_t *pm = hub_pwm_lookup(((hub_cfg_payload_t *)cmd.data_addr)->pin_code);
                if (!pm) {
                    rsp.status = HUB_RSP_ERR_UNKNOWN_TARGET;
                    break;
                }
                if (cmd.operation == HUB_OP_CONFIG) {
                    const hub_cfg_payload_t *cfg = (const hub_cfg_payload_t *)cmd.data_addr;
                    if (cfg->cfg == HUB_IOCT_CFG_AF) {
                        pm->mx_reinit();
                        HAL_TIM_PWM_Start(pm->htim, pm->ch);
                    } else {
                        hub_pwm_to_gpio_generic(pm, cfg);
                    }
                    rsp.status = HUB_RSP_OK;

                } else if (cmd.operation == HUB_OP_WRITE) {
                    /* payload: uint16_t duty (0~10000) */
                    uint16_t duty = *(uint16_t *)cmd.data_addr;
                    __HAL_TIM_SET_COMPARE(pm->htim, pm->ch, duty);
                    rsp.status = HUB_RSP_OK;
                } else if (cmd.operation == HUB_OP_READ) {
                    /* Current duty write-back data_addr (uint16_t) */
                    uint16_t *out = (uint16_t *)cmd.data_addr;
                    *out = (uint16_t)__HAL_TIM_GET_COMPARE(pm->htim, pm->ch);
                    rsp.len = sizeof(uint16_t);
                    rsp.status = HUB_RSP_OK;

                } else {
                    rsp.status = HUB_RSP_ERR_UNKNOWN_CMD;
                }
                break;
            }
            case HUB_TARGET_DAC:
                if (cmd.operation == HUB_OP_WRITE) {
                    rsp.status = HUB_RSP_OK;
                } else {
                    rsp.status = HUB_RSP_ERR_UNKNOWN_CMD;
                }
                break;
            case HUB_TARGET_WIFI:
                if (cmd.operation == HUB_OP_WRITE && cmd.len > 0) {
                    DEBUG_DUMP(IOT_LOG_DEBUG, "HUB_TARGET_WIFI: Transmitting %d bytes\r\n", cmd.len);
                    HAL_UART_Transmit_DMA(&huart7, (uint8_t*)cmd.data_addr, cmd.len);
                    rsp.status = HUB_RSP_OK;
                } else if(cmd.operation == HUB_OP_READ) {
                    DEBUG_DUMP(IOT_LOG_DEBUG, "HUB_TARGET_WIFI: Receiving %d bytes\r\n", cmd.len);
                    HAL_UART_Receive_DMA(&huart7, (uint8_t*)cmd.data_addr, cmd.len);
                    rsp.len = cmd.len;
                    rsp.data_addr = cmd.data_addr;
                    rsp.status = HUB_RSP_OK;
                } else {
                    rsp.status = HUB_RSP_ERR_UNKNOWN_CMD;
                }
                break;
            case HUB_TARGET_CAN:
                if (cmd.operation == HUB_OP_WRITE && cmd.len > 0) {
                    rsp.status = HUB_RSP_OK;
                } else {
                    rsp.status = HUB_RSP_ERR_UNKNOWN_CMD;
                }
                break;
            case HUB_TARGET_SD:
                if (cmd.operation == HUB_OP_READ || cmd.operation == HUB_OP_WRITE) {
                    // Implement SD read/write logic here
                    // This is a placeholder for actual SD read/write logic
                    rsp.status = HUB_RSP_OK;
                } else {
                    rsp.status = HUB_RSP_ERR_UNKNOWN_CMD;
                }
                break;
            default:
                rsp.status = HUB_RSP_ERR_UNKNOWN_TARGET;
                break;
        }
        tx_queue_send(&rsp_queue, &rsp, TX_NO_WAIT);
    }
}
