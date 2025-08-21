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

int w5500_modbus_start(const w5500_modbus_cfg_t *cfg);
void w5500_modbus_thread_entry(ULONG arg);

void w5500_modbus_server_helper();
#ifdef __cplusplus
}
#endif
#endif /* W5500_MODBUS_SERVER_H */
