// Example skeleton implementing an I2C communication hub
// using ThreadX threads and queues. This is not complete
// production code but demonstrates how the peripherals
// could be structured.

#include "tx_api.h"
#include "stm32h7xx_hal.h"
#include "i2c.h"      // HAL I2C handle
#include "usart.h"
#include "i2c_hub.h"
#include <string.h>
#include "iot.h"

#define HUB_I2C_ADDR 0x36
#define QUEUE_LEN    8
#define MSG_WORDS_CMD  ((sizeof(hub_cmd_t) + sizeof(ULONG) - 1) / sizeof(ULONG))
#define MSG_WORDS_RSP  ((sizeof(hub_rsp_t) + sizeof(ULONG) - 1) / sizeof(ULONG))
typedef struct {
    uint8_t target;      // e.g. enum for UART/SPI/GPIO
    uint8_t operation;   // read/write
    uint8_t data[16];    // example payload
} hub_cmd_t;

typedef struct {
    uint8_t status;
    uint8_t data[16];
} hub_rsp_t;

static TX_QUEUE cmd_queue;
static TX_QUEUE rsp_queue;
static TX_THREAD worker_thread;

static VOID worker_thread_entry(ULONG arg);

static hub_cmd_t  cmd_in;
static hub_rsp_t  rsp_out;

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t dir, uint16_t addr) {
    DEBUG_DUMP(IOT_LOG_DEBUG, "GET some thing\r\n");
    if (hi2c != &hi2c2 || addr != HUB_I2C_ADDR) return;
    DEBUG_DUMP(IOT_LOG_DEBUG, "GET some thing in 0x36\r\n");

    if (dir == I2C_DIRECTION_TRANSMIT) { 
        /**
         * Scan
         */
        if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_STOPF))
        {
            HAL_I2C_EnableListen_IT(hi2c);
            return;
        }
        HAL_I2C_Slave_Seq_Receive_DMA(hi2c,
            (uint8_t *)&cmd_in, sizeof(cmd_in), I2C_LAST_FRAME);
    } else {                             
        tx_queue_receive(&rsp_queue, &rsp_out, TX_NO_WAIT);
        HAL_I2C_Slave_Seq_Transmit_IT(hi2c,
            (uint8_t *)&rsp_out, sizeof(rsp_out), I2C_LAST_FRAME);
    }
}
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c != &hi2c2) return;
    DEBUG_DUMP(IOT_LOG_DEBUG, "Rx occour\r\n");

    tx_queue_send(&cmd_queue, &cmd_in, TX_NO_WAIT);
    HAL_I2C_EnableListen_IT(&hi2c2); 
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c != &hi2c2) return;
    DEBUG_DUMP(IOT_LOG_DEBUG, "Tx occour\r\n");

    HAL_I2C_EnableListen_IT(&hi2c2);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c != &hi2c2) return;
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_BERR | I2C_FLAG_ARLO | I2C_FLAG_AF);
    HAL_I2C_EnableListen_IT(&hi2c2);
}

void hub_start(void)
{
    static UCHAR cmd_queue_buf[QUEUE_LEN * sizeof(hub_cmd_t)];
    static UCHAR rsp_queue_buf[QUEUE_LEN * sizeof(hub_rsp_t)];
    static UCHAR uart_stack[1024];
    HAL_I2C_EnableListen_IT(&hi2c2);
    DEBUG_DUMP(IOT_LOG_DEBUG, "I2C INT Start.\r\n");
    tx_queue_create(&cmd_queue, "cmd_queue", MSG_WORDS_CMD,
                cmd_queue_buf, sizeof(cmd_queue_buf));
    tx_queue_create(&rsp_queue, "rsp_queue", MSG_WORDS_RSP,
                    rsp_queue_buf, sizeof(rsp_queue_buf));
    tx_thread_create(&worker_thread, "hub_worker", worker_thread_entry, 0,
                     uart_stack, sizeof(uart_stack), 5, 5,
                     TX_NO_TIME_SLICE, TX_AUTO_START);

    DEBUG_DUMP(IOT_LOG_INFO,
        "I2C hub ready, OAR1=0x%08lX (OA1=%02lX)\r\n",
        hi2c2.Instance->OAR1,
        (hi2c2.Instance->OAR1 >> 1) & 0x7F);
}

static void configure_gpio1_pullup(void)
{
    GPIO_InitTypeDef init = {0};
    init.Pin = GPIO_PIN_1;
    init.Mode = GPIO_MODE_INPUT;
    init.Pull = GPIO_PULLUP;
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_1);
    HAL_GPIO_Init(GPIOB, &init);
}

static VOID worker_thread_entry(ULONG arg)
{
    (void)arg;
    for (;;) {
        hub_cmd_t cmd;
        tx_queue_receive(&cmd_queue, &cmd_in, TX_WAIT_FOREVER);
        DEBUG_DUMP(IOT_LOG_DEBUG, "worker receive occour\r\n");
        
        hub_rsp_t rsp = {0};
        if (cmd.target == HUB_TARGET_UART) {
            // very basic handler example for UART target
            HAL_UART_Transmit(&huart5, cmd.data, strlen((char *)cmd.data), HAL_MAX_DELAY);
            rsp.status = 0; // ok
        } else if (cmd.target == HUB_TARGET_GPIO) {
            if (cmd.operation == HUB_GPIO_CFG_PULLUP && cmd.data[0] == 1) {
                configure_gpio1_pullup();
                rsp.status = 0;
            } else {
                rsp.status = 1; // unknown gpio command
            }
        } else {
            rsp.status = 1; // unknown target
        }
        tx_queue_send(&rsp_queue, &rsp, TX_WAIT_FOREVER);
    }
}
