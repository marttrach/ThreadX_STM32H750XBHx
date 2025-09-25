#include "modbus_rtu_client.h"
#include "tx_api.h"
#include "usart.h"

#define formosa_device_count 6

typedef struct {
    uint8_t  slave_addr;
    uint16_t qty;
    uint16_t summamry_len;
    uint16_t *summamry;
    uint16_t detail_len;
    uint16_t *detail;
} Formosa_Setting;

#ifndef modbus_formosa_h
#define modbus_formosa_h

extern Formosa_Setting formosa_setting[formosa_device_count];
extern Modbus_RTU_Frame formosa_rtu_req;
extern Modbus_RTU_Frame formosa_rtu_res;
extern uint8_t formosa_ups120;

#endif

void formosa_init_setting();
void formosa_set_ups120(uint8_t value);
void formosa_set_slave_addr(uint8_t *values);
void formosa_set_qty(uint8_t *values);
void formosa_reset_frame();
void formosa_build_request(uint8_t slave_addr, uint8_t function_code, uint16_t reg_addr, uint16_t reg_count);
void modbus_thread_start();
void modbus_thread_stop();