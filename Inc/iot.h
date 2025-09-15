#ifndef IOT_H
#define IOT_H

/* Debugging utilities */
#define IOT_LOG_NONE      0
#define IOT_LOG_ALL      1
#define IOT_LOG_DEBUG     2
#define IOT_LOG_INFO      3
#define IOT_LOG_WARNING   4
#define IOT_LOG_ERR       5
//Set debug level, 0 means no debug
#define IOT_DEBUG_LEVEL IOT_LOG_INFO

/* SDRAM */
#define SDRAM_START_ADDRESS 0xC0000000
#define HW_SDRAM_SIZE       (32 * 1024 * 1024)

#if (IOT_DEBUG_LEVEL ==  0)
#define DEBUG_DUMP(...)  do{}while(0)
#else
#include <stdio.h>
#if 0 /* Use this section for ThreadX debugging */
#include "tx_api.h"
static TX_MUTEX iot_debug_mutex;
#define DEBUG_DUMP(Lv, fmt, ...)                                               \
    do {                                                                       \
        if ((Lv) >= IOT_DEBUG_LEVEL) {                                         \
            static UCHAR once = 0;                                             \
            if (!once) {                                                       \
                tx_mutex_create(&iot_debug_mutex, (CHAR *)"dbg_mutex", TX_INHERIT); \
                once = 1;                                                      \
            }                                                                  \
            tx_mutex_get(&iot_debug_mutex, TX_WAIT_FOREVER);                   \
            TX_THREAD *cur = tx_thread_identify();                             \
            if (cur) {                                                         \
                printf("[%-12s] " fmt, (CHAR *)cur->tx_thread_name, ##__VA_ARGS__); \
            } else {                                                           \
                printf("[ISR         ] " fmt, ##__VA_ARGS__);                  \
            }                                                                  \
            tx_mutex_put(&iot_debug_mutex);                                    \
        }                                                                      \
    } while (0)
#else
#define DEBUG_DUMP(Lv, ...) do{if((Lv) >= IOT_DEBUG_LEVEL) printf(__VA_ARGS__);}while(0)
#endif /* IOT_DEBUG_LEVEL */
#endif
#endif
