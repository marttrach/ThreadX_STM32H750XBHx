#ifndef I2C_HUB_H
#define I2C_HUB_H

#include "stm32h7xx_hal.h"
#include "app_threadx.h"
#include "i2c_hub_mem.h"

#define ALIGN32(x)   (((x) + 31U) & ~31U)
#ifndef MIN
#define MIN(a, b)  (( (a) < (b) ) ? (a) : (b))
#endif

/**
 * I2C Hub definitions
 * This file defines the structure and functions for the I2C hub communication.
 */
#define HUB_I2C_ADDR 0x36
#define QUEUE_LEN    10
#define I2C_STAGE_Q_LEN   32U
#define I2C_SIZE_MTU 255

#ifndef HUB_ERR_FILL_BYTE
#define HUB_ERR_FILL_BYTE 0xFFu
#endif

/* Command targets */
#define HUB_TARGET_UART 0 //Now only UART 8
#define HUB_TARGET_GPIO 1 
#define HUB_TARGET_SPI  2
#define HUB_TARGET_I2C  3 //I2C1 -> UD3 UE3
#define HUB_TARGET_ADC  4
#define HUB_TARGET_PWM  5
#define HUB_TARGET_DAC  6
#define HUB_TARGET_WIFI 12 // UART 7
#define HUB_TARGET_CAN  15
#define HUB_TARGET_USB  16
#define HUB_TARGET_SD  17
#define HUB_TARGET_MEM 18 // Memory Check
#define HUB_SLAVE_I2C  19 // I2C2

/* CONTROL */
#define HUB_IOCT_CFG_SET 0
#define HUB_IOCT_CFG_GET 1
#define HUB_IOCT_CFG_TOGGLE 2
/* CONFIGURATION */
#define HUB_IOCT_CFG_NONE 0
#define HUB_IOCT_CFG_INPUT 1
#define HUB_IOCT_CFG_OUTPUT 2
#define HUB_IOCT_CFG_ANALOG 3
#define HUB_IOCT_CFG_AF 4
#define HUB_IOCT_CFG_PULLUP 5
#define HUB_IOCT_CFG_PULLDOWN 6
#define HUB_IOCT_CFG_OD 7

/**
 * Error codes for hub responses
 */
typedef uint8_t hub_rsp_status_t;
enum {
    HUB_RSP_OK = 0,
    HUB_RSP_ERR_UNKNOWN_TARGET = 1,
    HUB_RSP_ERR_UNKNOWN_CMD = 2,
    HUB_RSP_ERR_MEMORY = 3,
    HUB_RSP_ERR_CRC = 4,
    HUB_RSP_BUSY = 5,
    HUB_RSP_WORKER_ACK = 6,
    HUB_RSP_ERR_NONE = 7
};

/* Generic operations used by the state machine */
typedef uint8_t hub_operation_t;
enum {
    HUB_OP_NOP = 0,
    HUB_OP_WRITE = 1,
    HUB_OP_READ  = 2,
    HUB_OP_CONFIG = 3,
};

typedef enum {
    TX_STAGE_HDR = 0,
    TX_STAGE_PAY = 1,
} hub_tx_stage_t;

typedef enum {
    RX_STAGE_HDR = 0,
    RX_STAGE_PAY = 1,
} hub_rx_stage_t;

struct hub_tx_task_t_;
typedef struct hub_tx_task_t_ hub_tx_task_t;

struct hub_tx_task_t_ {
    uint8_t       *buf;
    uint32_t       total;
    uint32_t       sent;
    hub_tx_stage_t stage_tx;
    uint32_t       alloc_flags;
    void         (*done_cb)(hub_tx_task_t *);  /* ★ 這裡改用 hub_tx_task_t* */
    void          *user_ctx;
};

struct hub_rx_task_t_;
typedef struct hub_rx_task_t_ hub_rx_task_t;

struct hub_rx_task_t_{
    uint8_t  *buf;
    uint32_t  total;
    uint32_t  sent;
    hub_rx_stage_t stage_rx;
};

/* CMD */
typedef struct __attribute__((packed, aligned(4))) {
    uint8_t  target;
    hub_operation_t  operation;
    uint16_t len;          /* payload len (byte) */
    uint32_t data_addr;    /* start frame payload in sdram */
    uint8_t payload[]; /* Variable length payload */
} hub_cmd_t;

/* RSP */
typedef struct __attribute__((packed, aligned(4))) {
    hub_rsp_status_t  status;
    uint8_t  reserved;
    uint16_t len;
    uint32_t data_addr;
    uint8_t payload[]; /* Variable length payload */
} hub_rsp_t;

/* Error Event */
typedef struct __attribute__((packed, aligned(4))) {
    uint32_t        tag;       /* HUB_ERR_TAG */
    hub_rsp_status_t status;   /* HUB_RSP_ERR_* */
    uint32_t        data_addr; /* cmd header data_addr */
    void           *rx_ptr;
    uint32_t        rx_len;
} hub_err_evt_t;

typedef enum {
    RD_IDLE = 0,
    RD_PEEK,
    RD_DATA_WAIT,
    RD_DATA_READY_PAY
} hub_rd_mode_t;

/* PIN PACK */
typedef struct __attribute__((packed)){
    uint16_t pin_code;        /* Macro HUB_ENC_PIN() init */
    uint8_t  cfg;             /* HUB_IOCT_CFG_*       */
    uint8_t  af;              /* Alternate Function (0 = no change) */
} hub_cfg_payload_t;

/* Rx Tx Transaction Mark */
typedef struct {
    uint8_t  *ptr;
    uint32_t  total;
    uint32_t  head_mark;
    uint8_t   active;
} rx_txn_t;

#define CMD_HDR_SZ   (offsetof(hub_cmd_t, data_addr) + 4)   /* 8B */
#define RSP_HDR_SZ   (offsetof(hub_rsp_t, data_addr) + 4)   /* 8B*/
#define DUMMY_CHUNK  32
#define HUB_TASK_F_STATIC_BUF   (1u << 0)  /* buf static, no free */
#define HUB_TASK_F_STATIC_TASK  (1u << 1)  /* task static , no free */

#define QSIZE(arr)        ((ULONG)(sizeof(arr) / sizeof(ULONG)))
_Static_assert(sizeof(void*) == sizeof(ULONG), "Pointer size must equal ULONG on this port");
#define MSG_WORDS_PTR     ((UINT)TX_1_ULONG)

void iot_hub_start(void);
uint32_t iot_hub_crc32_hard(const uint8_t *buf, size_t len);
void iot_hub_tx_flush(void);
int hub_send_tx_frame(uint8_t *buf, uint32_t total);
int hub_send_tx_task(hub_tx_task_t *task);

#endif // I2C_HUB_H
