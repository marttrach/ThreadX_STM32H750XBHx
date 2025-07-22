// Example skeleton implementing an I2C communication hub
// using ThreadX threads and queues. This is not complete
// production code but demonstrates how the peripherals
// could be structured.

#include "i2c_hub.h"
#include "iot.h"
#include <string.h>
#include "usart.h"
#include "i2c.h"
#include "tx_api.h"


static TX_QUEUE cmd_queue;
static TX_QUEUE rsp_queue;
static TX_THREAD worker_thread;

static VOID worker_thread_entry(ULONG arg);

static hub_cmd_t  cmd_in;
static hub_rsp_t  rsp_out;

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c,
                          uint8_t dir, uint16_t addr7)
{
    if (hi2c != &hi2c2 || addr7 != HUB_I2C_ADDR) return;

    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);

    if (dir == I2C_DIRECTION_TRANSMIT)
    {
        HAL_I2C_Slave_Seq_Receive_DMA(hi2c,
            (uint8_t*)&cmd_in, sizeof(cmd_in), I2C_LAST_FRAME);
    }
    else
    {
        if (tx_queue_receive(&rsp_queue, &rsp_out, TX_NO_WAIT) != TX_SUCCESS)
            memset(&rsp_out, 0xFF, sizeof(rsp_out));

        HAL_I2C_Slave_Seq_Transmit_IT(hi2c,
            (uint8_t*)&rsp_out, sizeof(rsp_out), I2C_LAST_FRAME);
    }
}
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c != &hi2c2) return;

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
    static UCHAR worker_stack[1024];
    tx_queue_create(&cmd_queue, "cmd_queue", MSG_WORDS_CMD,
                cmd_queue_buf, sizeof(cmd_queue_buf));
    tx_queue_create(&rsp_queue, "rsp_queue", MSG_WORDS_RSP,
                    rsp_queue_buf, sizeof(rsp_queue_buf));
    tx_thread_create(&worker_thread, "hub_worker",
                 worker_thread_entry, 0,
                 worker_stack, sizeof(worker_stack),
                 10, 10, TX_NO_TIME_SLICE, TX_DONT_START);
    HAL_I2C_EnableListen_IT(&hi2c2);
    tx_thread_resume(&worker_thread);
}

static void configure_gpio2_pullup(void)
{
    GPIO_InitTypeDef init = {0};
    init.Pin = GPIO_PIN_12;
    init.Mode = GPIO_MODE_OUTPUT_PP;
    init.Pull = GPIO_NOPULL;
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_12);
    HAL_GPIO_Init(GPIOD, &init);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
}

static void configure_gpio2_pulldown(void)
{
    GPIO_InitTypeDef init = {0};
    init.Pin = GPIO_PIN_12;
    init.Mode = GPIO_MODE_OUTPUT_PP;
    init.Pull = GPIO_NOPULL;
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_12);
    HAL_GPIO_Init(GPIOD, &init);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
}

static VOID worker_thread_entry(ULONG arg)
{
    (void)arg;
    for (;;) {
        hub_cmd_t cmd;
        tx_queue_receive(&cmd_queue, &cmd, TX_WAIT_FOREVER);
        
        hub_rsp_t rsp = {0};
        if (cmd.target == HUB_TARGET_UART) {
            char test_msg[] = "Hello from I2C Hub\r\n";
            HAL_UART_Transmit_IT(&huart5, (uint8_t*)test_msg,
                                    sizeof(test_msg) - 1);
            rsp.status = HUB_RSP_OK;
        } else if (cmd.target == HUB_TARGET_GPIO) {
            if (cmd.operation == HUB_IOCT_CFG_PULLUP && cmd.data[0] == 1) {
                configure_gpio2_pullup();
                rsp.status = HUB_RSP_OK;
            } else if (cmd.operation == HUB_IOCT_CFG_PULLDOWN && cmd.data[0] == 1) {
                configure_gpio2_pulldown();
                rsp.status = HUB_RSP_OK;
            } else {
                rsp.status = HUB_RSP_ERR_UNKNOWN_CMD;
            }
        } else {
            rsp.status = HUB_RSP_ERR_UNKNOWN_TARGET;
        }
        tx_queue_send(&rsp_queue, &rsp, TX_NO_WAIT);
    }
}
