#ifndef W5500_MODBUS_SERVER_H
#define W5500_MODBUS_SERVER_H

#include <stdint.h>
#include "tx_api.h"

#ifndef MODBUS_TCP_PORT
#define MODBUS_TCP_PORT 502
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MODBUS_EXCEPTION_NONE               = 0x00,  // ✅ 無錯誤（非標準，內部使用）
    MODBUS_EXCEPTION_ILLEGAL_FUNCTION   = 0x01,  // 不支援的功能碼
    MODBUS_EXCEPTION_ILLEGAL_DATA_ADDR  = 0x02,  // 無效的資料地址
    MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE = 0x03,  // 無效的資料值
    MODBUS_EXCEPTION_SLAVE_DEVICE_FAIL  = 0x04,  // 裝置故障或無法執行請求
    MODBUS_EXCEPTION_ACKNOWLEDGE        = 0x05,  // 裝置接受但需延遲處理
    MODBUS_EXCEPTION_DEVICE_BUSY        = 0x06,  // 裝置忙碌中
    MODBUS_EXCEPTION_MEMORY_ERROR       = 0x08,  // 記憶體錯誤（如 parity error）
    MODBUS_EXCEPTION_GATEWAY_PATH       = 0x0A,  // Gateway 路徑不可達
    MODBUS_EXCEPTION_GATEWAY_TARGET     = 0x0B   // Gateway 目標裝置無回應
} Modbus_TCP_Exception_Code;

typedef struct {
    uint8_t listen_socket;
    uint8_t upstream_socket; //  1

    uint16_t listen_port;    // 502

    uint8_t bridge_mode;

    uint8_t upstream_ip[4]; /* if bridge_mode = 1*/
    uint16_t upstream_port;

    TX_THREAD *thread_obj;     /*NULL acceptable*/
    void      *thread_stack;   
    uint32_t   stack_size;     
    uint32_t   thread_prio;    /* 10~15 */
} w5500_modbus_cfg_t;

#pragma pack(push, 1)
typedef struct {
    uint16_t transaction_id;
    uint16_t protocol_id;
    uint16_t length;
    uint8_t  unit_id;
    uint8_t  function_code;
    uint8_t  data[252];  // 包含功能碼參數，例如起始地址、數量等
} ModbusTCPRequest;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
    uint16_t transaction_id;
    uint16_t protocol_id;
    uint16_t length;
    uint8_t  unit_id;
    uint8_t  function_code;
    uint8_t  byte_count;
    uint8_t  data[252];  // 最多 125 筆寄存器（125 × 2 = 250）
} ModbusTCPReply;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
    uint16_t transaction_id;
    uint16_t protocol_id;
    uint16_t length;
    uint8_t  unit_id;
    uint8_t  function_code;
    uint8_t  err_code;
} ModbusTCPError;
#pragma pack(pop)

int w5500_modbus_start(const w5500_modbus_cfg_t *cfg);
void w5500_modbus_thread_entry(ULONG arg);

void w5500_modbus_set_ip(uint8_t *ip);
void w5500_modbus_set_mask(uint8_t *mask);
void w5500_modbus_set_port(uint16_t port);
void w5500_modbus_thread_stop();
void w5500_modbus_server_helper();
#ifdef __cplusplus
}
#endif
#endif /* W5500_MODBUS_SERVER_H */
