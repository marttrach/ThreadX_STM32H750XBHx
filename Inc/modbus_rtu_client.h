#include <stdint.h>
#include <string.h>

typedef enum {
    MODBUS_READ_COILS           = 0x01,
    MODBUS_READ_DISCRETE_INPUTS = 0x02,
    MODBUS_READ_HOLDING_REGS    = 0x03,
    MODBUS_READ_INPUT_REGS      = 0x04,
    MODBUS_WRITE_SINGLE_COIL    = 0x05,
    MODBUS_WRITE_SINGLE_REG     = 0x06,
    MODBUS_WRITE_MULTIPLE_COILS = 0x0F,
    MODBUS_WRITE_MULTIPLE_REGS  = 0x10
} Modbus_Func_Code;

typedef enum {
    MODBUS_EX_OK = 0,
    MODBUS_EX_ILLEGAL_FUNCTION = 0x01,
    MODBUS_EX_ILLEGAL_ADDRESS  = 0x02,
    MODBUS_EX_ILLEGAL_VALUE    = 0x03,
    MODBUS_EX_DEVICE_FAILURE   = 0x04,
    MODBUS_EX_DEVICE_BUSY      = 0x06,
    MODBUS_EX_GATEWAY_FAILED   = 0x0B,
    MODBUS_EX_CRC_ERROR        = 0xFE,
    MODBUS_EX_FORMAT_ERROR     = 0xFF
} Modbus_RTU_Exception_Code;

typedef struct {
    uint8_t  slave_addr;       // 從機地址
    uint8_t  function_code;    // 功能碼
    uint16_t reg_addr;         // 起始寄存器地址
    uint16_t reg_count;        // 寄存器數量（讀取或寫入）
    uint8_t  byte_count;       // 資料區位元組數（回應用）
    uint8_t  data[250];        // 資料區（最多 125 個寄存器）
    uint16_t crc;              // CRC 校驗值
    uint8_t  frame[256];       // 原始封包（含 CRC）
    uint16_t frame_len;        // 封包長度
} Modbus_RTU_Frame;

#ifndef modbus_rtu_client_h
#define modbus_rtu_client_h

extern uint8_t MODBUS_CRC_LENGTH;
extern uint8_t MODBUS_ERROR_BIAS;
extern uint8_t MODBUS_RESPONSE_HDR_LENGTH;
extern uint8_t MODBUS_ERROR_RESP_LEN;
extern uint8_t MODBUS_FIXED_RESP_LEN;
extern uint8_t MODBUS_MBAP_HDR_LENGTH;

#endif

void modbus_build_request(Modbus_RTU_Frame *req);
uint8_t modbus_parse_response(Modbus_RTU_Frame *resp, const uint8_t *raw, uint16_t len);