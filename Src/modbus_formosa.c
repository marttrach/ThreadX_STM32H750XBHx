#include "i2c_hub.h"
#include "modbus_formosa.h"
#include "app_threadx.h"

Formosa_Setting formosa_setting[formosa_device_count];
Modbus_RTU_Frame formosa_rtu_req;
Modbus_RTU_Frame formosa_rtu_res;
uint8_t formosa_ups120 = 0;

// threadX definition
TX_THREAD modbus_thread;
UCHAR uart_thread_stack[2048];
uint8_t uart_rx_buffer[256];
uint16_t uart_rx_len = 0;
uint16_t uart_wait_ms = 250;

void formosa_init_setting() {
    for (int i = 0; i < formosa_device_count; i++) {
        formosa_setting[i].slave_addr = 0;
        formosa_setting[i].qty = 0;
        
        formosa_setting[i].summamry_len = sizeof(uint16_t) * 11;
        if (formosa_setting[i].summamry != NULL) {
            free(formosa_setting[i].summamry);
        }
        formosa_setting[i].summamry = NULL;
        formosa_setting[i].summamry = malloc(formosa_setting[i].summamry_len);
        memset(formosa_setting[i].summamry, 0, formosa_setting[i].summamry_len);
        
        formosa_setting[i].detail_len = 0;
        if (formosa_setting[i].detail != NULL) {
            free(formosa_setting[i].detail);
        }
        formosa_setting[i].detail = NULL;
    }
    
    formosa_reset_frame(&formosa_rtu_req);
    formosa_reset_frame(&formosa_rtu_res);
}

void formosa_set_ups120(uint8_t value) {
    formosa_ups120 = value;
}

void formosa_set_slave_addr(uint8_t *values) {
    for (int i = 0; i < formosa_device_count; i++) {
        formosa_setting[i].slave_addr = values[i];
    }
}

void formosa_set_qty(uint8_t *values) {
    for (int i = 0; i < formosa_device_count; i++) {
        formosa_setting[i].qty = values[i] * 8;
        formosa_setting[i].detail_len = sizeof(uint16_t) * formosa_setting[i].qty;
        if (formosa_setting[i].qty > 0) {
            formosa_setting[i].detail = realloc(formosa_setting[i].detail, formosa_setting[i].detail_len);
            memset(formosa_setting[i].detail, 0, formosa_setting[i].detail_len);
        }
        else if (formosa_setting[i].detail != NULL) {
            free(formosa_setting[i].detail);
            formosa_setting[i].detail = NULL;
        }
    }
}

void formosa_reset_frame(Modbus_RTU_Frame* frame) {
    memset(frame, 0, sizeof(Modbus_RTU_Frame));
}

void formosa_build_request(uint8_t slave_addr, uint8_t function_code, uint16_t reg_addr, uint16_t reg_count) {
    formosa_reset_frame(&formosa_rtu_req);

    formosa_rtu_req.slave_addr = slave_addr;
    formosa_rtu_req.function_code = function_code;
    formosa_rtu_req.reg_addr = reg_addr;
    formosa_rtu_req.reg_count = reg_count;
    modbus_build_request(&formosa_rtu_req);
}

uint8_t formosa_parse_response(const uint8_t *raw, uint16_t len) {
    formosa_reset_frame(&formosa_rtu_res);
    
    uint8_t res = modbus_parse_response(&formosa_rtu_res, raw, len);
    DEBUG_DUMP(IOT_LOG_INFO, "formosa_parse_response, res: %d\r\n", res);
    return res;
}

uint8_t exit_read(uint8_t *res, u_int16_t len) {
    if (len < MODBUS_ERROR_RESP_LEN)
        return 0;
    else if (res[1] >= MODBUS_ERROR_BIAS)
        return 0;
    else if (MODBUS_READ_COILS <= res[1] && res[1] <= MODBUS_READ_INPUT_REGS) {
        u_int16_t expected_len = MODBUS_RESPONSE_HDR_LENGTH + 1 + res[2] + MODBUS_CRC_LENGTH;
        if (len < expected_len) return 0;
    }
    else if (len < MODBUS_FIXED_RESP_LEN)
        return 0;
    return 1;
}

void uart_read_blocking() {
    memset(uart_rx_buffer, 0, sizeof(uart_rx_buffer));
    uart_rx_len = 0;
    
    u_int32_t start_time = HAL_GetTick();
    while (HAL_GetTick() - start_time < uart_wait_ms) {
        uint8_t byte;
        if (__HAL_UART_GET_FLAG(&huart8, UART_FLAG_RXNE)) {
            while (__HAL_UART_GET_FLAG(&huart8, UART_FLAG_RXNE)) {
                if (HAL_UART_Receive(&huart8, &byte, 1, 100) == HAL_OK) {
                    if (uart_rx_len < sizeof(uart_rx_buffer)) uart_rx_buffer[uart_rx_len++] = byte;
                }
            }
            if (exit_read(uart_rx_buffer, uart_rx_len) == 1) break;
            start_time = HAL_GetTick();
        }
        tx_thread_sleep(0.1);
    }
    // DEBUG_DUMP(IOT_LOG_DEBUG, "rs485 read. length: %d\r\n", uart_rx_len);
}

void modbus_thread_entry(ULONG input) {
    // 清除未處裡的資料
    while (__HAL_UART_GET_FLAG(&huart8, UART_FLAG_RXNE)) {
        volatile uint8_t unHandle = (uint8_t)(huart8.Instance->RDR);  // 讀出資料但不處理
        tx_thread_sleep(0.1);
    }

    while (1) {
        for (int i = 0; i < formosa_device_count; i++) {
            if (formosa_setting[i].slave_addr > 0) {
                DEBUG_DUMP(IOT_LOG_INFO, "formosa handle device. addr: %d\r\n", formosa_setting[i].slave_addr);
                // query device summary data
                formosa_reset_frame(&formosa_rtu_req);
                if (formosa_ups120 == 0) 
                    formosa_build_request(formosa_setting[i].slave_addr, 3, 1440, 11);
                else
                    formosa_build_request(formosa_setting[i].slave_addr, 3, 1476, 11);
                HAL_UART_Transmit(&huart8, formosa_rtu_req.frame, formosa_rtu_req.frame_len, HAL_MAX_DELAY);
                // DEBUG_DUMP(IOT_LOG_DEBUG, "formosa req. reg: %d\r\n", formosa_rtu_req.reg_addr);
                tx_thread_sleep(0.1);

                uart_read_blocking();
                if (formosa_parse_response(uart_rx_buffer, uart_rx_len) == MODBUS_EX_OK) {
                    memcpy(formosa_setting[i].summamry, formosa_rtu_res.data, formosa_rtu_res.byte_count);
                    // DEBUG_DUMP(IOT_LOG_DEBUG, "set summamry. length: %d\r\n", formosa_rtu_res.byte_count);
                }
                else {
                    memset(formosa_setting[i].summamry, 0, formosa_setting[i].summamry_len);
                    // DEBUG_DUMP(IOT_LOG_DEBUG, "erase summamry. length: %d\r\n", formosa_rtu_res.byte_count);
                }
                tx_thread_sleep(0.1);

                // query device detail data
                uint8_t interval = 120;
                uint8_t times = formosa_setting[i].qty > 0 ? (formosa_setting[i].qty / interval) + 1 : 0;
                uint8_t rest = formosa_setting[i].qty % interval;
                if (rest == 0) rest = interval;
                for (int j = 0; j < times; j++) {
                    formosa_reset_frame(&formosa_rtu_req);
                    uint16_t offset = j * interval;
                    if (j == times - 1)
                        formosa_build_request(formosa_setting[i].slave_addr, 3, 512 + offset, rest);
                    else
                        formosa_build_request(formosa_setting[i].slave_addr, 3, 512 + offset, interval);
                    HAL_UART_Transmit(&huart8, formosa_rtu_req.frame, formosa_rtu_req.frame_len, HAL_MAX_DELAY);
                    // DEBUG_DUMP(IOT_LOG_DEBUG, "formosa req. reg: %d\r\n", formosa_rtu_req.reg_addr);
                    
                    tx_thread_sleep(0.1);
                    uart_read_blocking();
                    if (formosa_parse_response(uart_rx_buffer, uart_rx_len) == MODBUS_EX_OK && formosa_setting[i].detail_len >= offset + formosa_rtu_res.byte_count) {
                        memcpy(formosa_setting[i].detail + offset, formosa_rtu_res.data, formosa_rtu_res.byte_count);
                        // DEBUG_DUMP(IOT_LOG_DEBUG, "set detail. index: %d length: %d\r\n", offset, formosa_rtu_res.byte_count);
                    }
                    else {
                        memset(formosa_setting[i].detail, 0, formosa_setting[i].detail_len);
                        // DEBUG_DUMP(IOT_LOG_DEBUG, "erase detail. index: %d length: %d\r\n", offset, formosa_rtu_res.byte_count);
                    }
                    tx_thread_sleep(0.1);
                }
            }
        }
    }
}

void modbus_thread_start() {
    tx_thread_create(&modbus_thread, "Modbus Thread", modbus_thread_entry, MSG_WORDS_PTR,
                     uart_thread_stack, QSIZE(uart_thread_stack),
                     12, 12, TX_NO_TIME_SLICE, TX_AUTO_START);
}

void modbus_thread_stop() {
    UINT state;
    CHAR *name;
    if (tx_thread_info_get(&modbus_thread, &name, &state, TX_NULL, TX_NULL, TX_NULL, TX_NULL, TX_NULL, TX_NULL) == TX_SUCCESS) {
        if (state == TX_READY) {
            tx_thread_terminate(&modbus_thread);
            tx_thread_delete(&modbus_thread);
        }
    }
}
