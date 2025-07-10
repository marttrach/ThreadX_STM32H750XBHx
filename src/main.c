#include "tx_api.h"
#include "stm32h7xx_hal.h"
#include <stdio.h>
#include <stdint.h>

UART_HandleTypeDef huart5;

void SystemClock_Config(void);
static void MX_UART5_Init(void);

void tx_application_define(void *first_unused_memory)
{
    (void)first_unused_memory;
    /* Send hello world once threadX is running */
    const char msg[] = "Hello world\r\n";
    HAL_UART_Transmit(&huart5, (uint8_t*)msg, sizeof(msg) - 1, HAL_MAX_DELAY);
    while(1) { tx_thread_sleep(TX_TIMER_TICKS_PER_SECOND); }
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_UART5_Init();

    tx_kernel_enter();
}

void SystemClock_Config(void)
{
    /* Placeholder: configure HCLK and PCLK. Actual implementation depends on board. */
}

static void MX_UART5_Init(void)
{
    __HAL_RCC_UART5_CLK_ENABLE();
    huart5.Instance = UART5;
    huart5.Init.BaudRate = 115200;
    huart5.Init.WordLength = UART_WORDLENGTH_8B;
    huart5.Init.StopBits = UART_STOPBITS_1;
    huart5.Init.Parity = UART_PARITY_NONE;
    huart5.Init.Mode = UART_MODE_TX_RX;
    huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart5.Init.OverSampling = UART_OVERSAMPLING_16;
    huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    HAL_UART_Init(&huart5);
}

int _write(int fd, char *ptr, int len)
{
    if (fd == 1 || fd == 2) {
        HAL_UART_Transmit(&huart5, (uint8_t*)ptr, len, HAL_MAX_DELAY);
        return len;
    }
    return -1;
}
