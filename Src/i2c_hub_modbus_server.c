#include "i2c_hub_modbus_server.h"
#include "i2c_hub_w55k.h"

// WIZnet ioLibrary
#include "wizchip_conf.h"
#include "socket.h"
#include "i2c_hub.h"
#include "spi.h"
#include "modbus_formosa.h"

#ifndef MODBUS_BUF_SZ
#define MODBUS_BUF_SZ 1460   /* one MSS */
#endif

static TX_THREAD s_modbus_thread;
static UCHAR *s_modbus_stack = NULL;
static uint8_t *g_buf_c2u = NULL;
static uint8_t *g_buf_u2c = NULL;
static uint32_t g_buf_sz  = 0;
static w5500_modbus_cfg_t s_cfg;

/* w55k pin config  */
static w5500_port_cfg_t pcfg = {
    .hspi     = &hspi1,
    .cs_port  = GPIOG, .cs_pin  = GPIO_PIN_10,  /* G10 */
    .rst_port = GPIOH, .rst_pin = GPIO_PIN_4,   /* H4 */
    .int_port = NULL,  .int_pin = 0,            /* D11 in while loop maybe no need */ 
    // .int_port = GPIOD,   .int_pin = GPIO_PIN_11, /* D11*/
    .mutex_inherit = TX_NO_INHERIT,
};

/* network config - no dhcp */
static w5500_net_cfg_t n_cfg = {
    .mac = { W5500_DEF_MAC0 },
    .ip  = { W5500_DEF_IP },
    .mask= { W5500_DEF_MASK },
    .gw  = { W5500_DEF_GW },
    .dns = { W5500_DEF_DNS },
    .use_dhcp = 0,
};

/* modbus tcp config */
static w5500_modbus_cfg_t m_cfg = {
    .listen_socket   = 0,
    .upstream_socket = 1,
    .listen_port     = 502,
    .bridge_mode     = 0,     // 0=echo(test); 1=trans
    .upstream_ip     = {0,0,0,0},
    .upstream_port   = 0,
    .thread_obj      = NULL,
    .thread_stack    = NULL,
    .stack_size      = 0,
    .thread_prio     = 12,
};

void w5500_modbus_set_ip(uint8_t *ip) {
    memcpy(n_cfg.ip, ip, 4);
    DEBUG_DUMP(IOT_LOG_INFO, "w550 set ip: %d.%d.%d.%d\r\n", n_cfg.ip[0], n_cfg.ip[1], n_cfg.ip[2], n_cfg.ip[3]);
}

void w5500_modbus_set_mask(uint8_t *mask) {
    memcpy(n_cfg.mask, mask, 4);
    DEBUG_DUMP(IOT_LOG_INFO, "w550 set mask: %d.%d.%d.%d\r\n", n_cfg.mask[0], n_cfg.mask[1], n_cfg.mask[2], n_cfg.mask[3]);
}

void w5500_modbus_set_port(uint16_t port) {
    m_cfg.listen_port = port;
    DEBUG_DUMP(IOT_LOG_INFO, "w550 set listen port: %d\r\n", m_cfg.listen_port);
}

static int open_listen(uint8_t sn, uint16_t port)
{
    int8_t ret;
    if ((ret = socket(sn, Sn_MR_TCP, port, SF_TCP_NODELAY)) != sn) return -1;
    if ((ret = listen(sn)) != SOCK_OK) { close(sn); return -2; }
    setRTR(2000); // 200ms
    setRCR(5);
    return 0;
}

static int open_connect(uint8_t sn, const uint8_t ip[4], uint16_t port)
{
    int8_t ret;
    if ((ret = socket(sn, Sn_MR_TCP, 0, 0)) != sn) return -1;
    if ((ret = connect(sn, (uint8_t*)ip, port)) != SOCK_OK) { close(sn); return -2; }
    for (int i=0; i<200; ++i) {
        if (getSn_SR(sn) == SOCK_ESTABLISHED) return 0;
        w5500_delay_ms(10);
    }
    close(sn);
    return -3;
}

static void close_if_open(uint8_t sn)
{
    uint8_t sr = getSn_SR(sn);
    if (sr != SOCK_CLOSED) close(sn);
}

static void bridge_loop(uint8_t sn_cli, uint8_t sn_up)
{
    g_buf_sz  = HUB_ALIGN_UP(MODBUS_BUF_SZ);
    if (!g_buf_c2u) g_buf_c2u = (uint8_t*)hub_heap_alloc_aligned(g_buf_sz, HUB_DMA_ALIGN);
    if (!g_buf_u2c) g_buf_u2c = (uint8_t*)hub_heap_alloc_aligned(g_buf_sz, HUB_DMA_ALIGN);
    if (!g_buf_c2u || !g_buf_u2c) return;
    while (1) {
        uint8_t sr_cli = getSn_SR(sn_cli);
        uint8_t sr_up  = getSn_SR(sn_up);
        if (sr_cli != SOCK_ESTABLISHED || sr_up != SOCK_ESTABLISHED) break;

        uint16_t rsr_cli = getSn_RX_RSR(sn_cli);
        uint16_t rsr_up  = getSn_RX_RSR(sn_up);

        if (rsr_cli) {
            int32_t r = recv(sn_cli, g_buf_c2u, (rsr_cli > g_buf_sz) ? g_buf_sz : rsr_cli);
            if (r <= 0) break;
            (void)send(sn_up, g_buf_c2u, (uint16_t)r);
        }

        if (rsr_up) {
            int32_t r = recv(sn_up, g_buf_u2c, (rsr_up > g_buf_sz) ? g_buf_sz : rsr_up);
            if (r <= 0) break;
            (void)send(sn_cli, g_buf_u2c, (uint16_t)r);
        }

        tx_thread_sleep(MS_TO_TICKS(1));
    }
}

static int tcp_send_all_relaxed(uint8_t sn, const uint8_t *buf, uint16_t len, UINT max_wait_ms)
{
    UINT waited = 0;
    uint16_t sent = 0;

    while (sent < len) {
        int32_t s = send(sn, (uint8_t*)buf + sent, (uint16_t)(len - sent));
        if (s > 0) {
            sent += (uint16_t)s;
            continue;
        }
        if (s == 0) {
            uint8_t sr = getSn_SR(sn);
            if (sr != SOCK_ESTABLISHED && sr != SOCK_CLOSE_WAIT) return -12;
            uint8_t ir = getSn_IR(sn);
            if (ir & Sn_IR_TIMEOUT) { setSn_IR(sn, Sn_IR_TIMEOUT); return -11; }
            if (waited >= max_wait_ms) return -10;
            tx_thread_sleep(MS_TO_TICKS(1));
            waited += 1;
            continue;
        }
        return (int)s;
    }
    return 0;
}

uint16_t modbus_get_u16_be(uint8_t high, uint8_t low) {
    return ((uint16_t)high << 8) | low;
}

uint16_t modbus_set_u16_be(uint16_t val) {
    return (val << 8) | (val >> 8);
}

// 檢查封包長度是否符合 Length 欄位定義
uint8_t modbus_check_tcp_packet_validity(uint8_t *packet, uint16_t actual_len) {
    if (actual_len < 7) return 0;  // Header 最少 7 bytes

    uint16_t len_field = modbus_get_u16_be(packet[4], packet[5]);
    uint16_t expected_len = 6 + len_field;  // 6 bytes header + len_field

    return actual_len >= expected_len ? 1 : 0;
}

int reply_modbus_tcp_error(uint8_t sn_cli, ModbusTCPRequest *req, uint8_t err_code) {
    DEBUG_DUMP(IOT_LOG_ERR, "modbus tcp reply error, err: %d\r\n", err_code);
    
    ModbusTCPError reply;
    reply.transaction_id = req->transaction_id;
    reply.protocol_id = req->protocol_id;
    reply.length = modbus_set_u16_be(3);
    reply.unit_id = req->unit_id;
    reply.function_code = req->function_code | 0x80;
    reply.err_code = err_code;

    return tcp_send_all_relaxed(sn_cli, (uint8_t *)&reply, (uint16_t)sizeof(reply), 5000);
}

uint16_t build_modbus_reply(ModbusTCPReply *reply, ModbusTCPRequest *req, uint16_t *registers, uint16_t start_addr, uint16_t quantity) {
    if (quantity == 0 || quantity > 125) return 0;

    reply->transaction_id = req->transaction_id;
    reply->protocol_id    = 0x0000;
    reply->length         = modbus_set_u16_be(3 + quantity * 2);  // Unit ID + Function + Byte Count + Data --> Big Endian
    reply->unit_id        = req->unit_id;
    reply->function_code  = req->function_code;
    reply->byte_count     = quantity * 2;
    memcpy(reply->data, &registers[start_addr], reply->byte_count);
    
    return sizeof(ModbusTCPReply) - sizeof(reply->data) + reply->byte_count;
}

int parse_modbus_tcp_request(uint8_t sn_cli, uint8_t *packet, uint16_t len) {
    if (modbus_check_tcp_packet_validity(packet, len) == 0) {
        // printf("❌ 封包長度不合法\n");
        DEBUG_DUMP(IOT_LOG_ERR, "modbus tcp request wrong length, length: %d\r\n", len);
        return 0;
    }

    ModbusTCPRequest *req = (ModbusTCPRequest *)packet;

    // uint16_t transaction_id = modbus_get_u16_be(packet[0], packet[1]);
    // uint16_t protocol_id    = modbus_get_u16_be(packet[2], packet[3]);
    // uint16_t length_field   = modbus_get_u16_be(packet[4], packet[5]);

    // printf("Transaction ID: 0x%04X\n", transaction_id);
    // printf("Protocol ID:    0x%04X\n", protocol_id);
    // printf("Length:         %d\n", length_field);
    // printf("Unit ID:        %d\n", req->unit_id);
    // printf("Function Code:  0x%02X\n", req->function_code);

    if (req->function_code == 0x03) {
        if (len >= 12) {
            uint16_t start_addr = modbus_get_u16_be(req->data[0], req->data[1]);
            uint16_t quantity   = modbus_get_u16_be(req->data[2], req->data[3]);
            if (quantity == 0) {
                return reply_modbus_tcp_error(sn_cli, req, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDR);
            }

            for (int i = 0; i < formosa_device_count; i++) {
                if (formosa_setting[i].slave_addr == req->unit_id) {
                    ModbusTCPReply reply;
                    uint16_t reply_len = 0;

                    if (start_addr >= 512 && start_addr + quantity <= 512 + formosa_setting[i].qty) {
                        reply_len = build_modbus_reply(&reply, req, formosa_setting[i].detail, start_addr - 512, quantity);
                    }
                    else if (formosa_ups120 == 0 && start_addr >= 1440 && start_addr + quantity <= 1440 + 11) {
                        reply_len = build_modbus_reply(&reply, req, formosa_setting[i].summamry, start_addr - 1440, quantity);
                    }
                    else if (formosa_ups120 == 1 && start_addr >= 1440 && start_addr + quantity <= 1476 + 11) {
                        reply_len = build_modbus_reply(&reply, req, formosa_setting[i].summamry, start_addr - 1476, quantity);
                    }
                    else {
                        return reply_modbus_tcp_error(sn_cli, req, MODBUS_EXCEPTION_ILLEGAL_DATA_ADDR);
                    }
                    return tcp_send_all_relaxed(sn_cli, (uint8_t *)&reply, reply_len, 5000);
                }
            }
        }
    }
    else {
        return reply_modbus_tcp_error(sn_cli, req, MODBUS_EXCEPTION_ILLEGAL_FUNCTION);
    }
    return 0;
}


static void echo_loop(uint8_t sn_cli)
{
    if (!g_buf_c2u || g_buf_sz == 0) return;
    DEBUG_DUMP(IOT_LOG_INFO, "echo loop. start sn: %d\r\n", sn_cli);

    setSn_KPALVTR(sn_cli, 2);

    while (1) {
        uint8_t sr = getSn_SR(sn_cli);

        if (sr == SOCK_ESTABLISHED) {
            uint8_t ir = getSn_IR(sn_cli);
            if (ir & Sn_IR_CON) setSn_IR(sn_cli, Sn_IR_CON);
        }
        else if (sr == SOCK_SYNRECV) {
            DEBUG_DUMP(IOT_LOG_INFO, "echo loop wait handshake. sr: 0x%02X\r\n", sr);
            tx_thread_sleep(MS_TO_TICKS(50));
        }
        else if (sr == SOCK_CLOSE_WAIT) {
            uint16_t rsr = getSn_RX_RSR(sn_cli);
            DEBUG_DUMP(IOT_LOG_INFO, "echo loop. start rsr: %d\r\n", rsr);
            if (rsr) {
                if (rsr > g_buf_sz) rsr = g_buf_sz;
                int32_t r = recv(sn_cli, g_buf_c2u, rsr);
                DEBUG_DUMP(IOT_LOG_INFO, "echo loop. start recv: %ld\r\n", (long)r);
                // if (r > 0) (void)tcp_send_all_relaxed(sn_cli, g_buf_c2u, (uint16_t)r, 5000);
                if (r > 0) (void)parse_modbus_tcp_request(sn_cli, g_buf_c2u, (uint16_t)r);
            }
            disconnect(sn_cli);
            DEBUG_DUMP(IOT_LOG_INFO, "echo loop disconnect. sn: %d\r\n", sn_cli);
            break;
        }
        else {
            DEBUG_DUMP(IOT_LOG_INFO, "echo loop break. sn: %d, sr: 0x%02X\r\n", sn_cli, sr);
            break;
        }

        uint16_t rsr = getSn_RX_RSR(sn_cli);
        if (rsr) {
            if (rsr > g_buf_sz) rsr = g_buf_sz;
            int32_t r = recv(sn_cli, g_buf_c2u, rsr);
            if (r <= 0) {
                if (r < 0) DEBUG_DUMP(IOT_LOG_ERR, "echo loop err. sn: %d, recv: %ld\r\n", sn_cli, (long)r);
                break;
            }
            // int rc = tcp_send_all_relaxed(sn_cli, g_buf_c2u, (uint16_t)r, 5000);
            int rc = parse_modbus_tcp_request(sn_cli, g_buf_c2u, (uint16_t)r);
            if (rc != 0) {
                DEBUG_DUMP(IOT_LOG_ERR, "echo loop err. modbus handle sn: %d, rc: %d\r\n", sn_cli, rc);
                break;
            }
        } else {
            tx_thread_sleep(MS_TO_TICKS(1));
        }
    }
}

void w5500_modbus_thread_entry(ULONG arg)
{
    const w5500_modbus_cfg_t *cfg = (const w5500_modbus_cfg_t *)arg;
    const uint8_t sn_cli = cfg->listen_socket;
    const uint8_t sn_up  = cfg->upstream_socket;

    for (;;) {
        if (open_listen(sn_cli, cfg->listen_port ? cfg->listen_port : MODBUS_TCP_PORT) != 0) {
            w5500_delay_ms(200);
            continue;
        }
        while (getSn_SR(sn_cli) == SOCK_LISTEN || getSn_SR(sn_cli) == SOCK_INIT) {
            tx_thread_sleep(MS_TO_TICKS(10));
        }
        if (getSn_SR(sn_cli) != SOCK_ESTABLISHED && getSn_SR(sn_cli) != SOCK_SYNRECV) {
            close_if_open(sn_cli);
            DEBUG_DUMP(IOT_LOG_ERR, "w5500_modbus_thread_entry not established. listen socket: 0x%x state: 0x%x\r\n", sn_cli, getSn_SR(sn_cli));
            continue;
        }

        int bridged = 0;
        if (cfg->bridge_mode) {
            if (open_connect(sn_up, cfg->upstream_ip, cfg->upstream_port) == 0) {
                bridged = 1;
            }
        }
        if (bridged) {
            bridge_loop(sn_cli, sn_up);
            close_if_open(sn_up);
        } else {
            echo_loop(sn_cli);
            DEBUG_DUMP(IOT_LOG_ALL, "w5500_modbus_thread_entry: Closing client socket %d\r\n", sn_cli);
        }
        DEBUG_DUMP(IOT_LOG_ALL, "w5500_modbus_thread_entry: Closing client socket %d\r\n", sn_cli);
        close_if_open(sn_cli);
    }
}

int w5500_modbus_start(const w5500_modbus_cfg_t *cfg) {
    if (!cfg) return -1;
    s_cfg = *cfg;
    DEBUG_DUMP(IOT_LOG_INFO, 
        "w5500_modbus_start. listen_socket: %d, upstream_socket: %d, listen_port: %d, bridge_mode: %d\r\n",
        s_cfg.listen_socket, s_cfg.upstream_socket, s_cfg.listen_port, s_cfg.bridge_mode);
    
    uint32_t want_stack = cfg->stack_size ? cfg->stack_size : 4096;
    uint32_t stack_sz   = HUB_ALIGN_UP(want_stack);

    if (cfg->thread_stack == NULL) {
        if (!s_modbus_stack) {
            s_modbus_stack = (UCHAR*)hub_heap_alloc_aligned(stack_sz, HUB_DMA_ALIGN);
            if (!s_modbus_stack) return -3; // out of memory
        }
    }

    g_buf_sz = HUB_ALIGN_UP(MODBUS_BUF_SZ);
    if (!g_buf_c2u) g_buf_c2u = (uint8_t*)hub_heap_alloc_aligned(g_buf_sz, HUB_DMA_ALIGN);
    if (s_cfg.bridge_mode && !g_buf_u2c)
        g_buf_u2c = (uint8_t*)hub_heap_alloc_aligned(g_buf_sz, HUB_DMA_ALIGN);
    if (!g_buf_c2u || (s_cfg.bridge_mode && !g_buf_u2c)) return -4;

    TX_THREAD *thr  = cfg->thread_obj   ? cfg->thread_obj  : &s_modbus_thread;
    void      *stk  = cfg->thread_stack ? cfg->thread_stack: s_modbus_stack;
    uint32_t   stksz= cfg->thread_stack ? cfg->stack_size  : stack_sz;
    uint32_t   prio = cfg->thread_prio  ? cfg->thread_prio : 12;

    UINT rc = tx_thread_create(thr, "w5500_modbus",
                               w5500_modbus_thread_entry,
                               (ULONG)&s_cfg,
                               stk, stksz,
                               prio, prio,
                               TX_NO_TIME_SLICE, TX_AUTO_START);
    if (rc != TX_SUCCESS) {
        DEBUG_DUMP(IOT_LOG_INFO, "tx thread create err. rc: %u, size: %lu\r\n", rc, (unsigned long)stksz);
        return -2;
    }
    return 0;
}

void w5500_modbus_thread_stop() {
    UINT state;
    CHAR *name;
    UINT res = 0;
    TX_THREAD *thr = m_cfg.thread_obj ? m_cfg.thread_obj : &s_modbus_thread;
    
    res = tx_thread_info_get(thr, &name, &state, TX_NULL, TX_NULL, TX_NULL, TX_NULL, TX_NULL, TX_NULL);
    if (res == TX_SUCCESS) {
        res = tx_thread_terminate(thr);
        res = tx_thread_delete(thr);
    }
    w5500_port_deinit(&pcfg);
}

void w5500_modbus_server_helper() {
    int res = w5500_port_init(&pcfg);
    if (res != 0) {
        DEBUG_DUMP(IOT_LOG_DEBUG, "w5500_modbus init failed. res: %d\r\n", res);
        return;
    }
    
    DEBUG_DUMP(IOT_LOG_INFO, "w5500_modbus helper.\r\n");
    if (w5500_bringup(&n_cfg) != 0) {
        /* plug */
        return;
    }
    DEBUG_DUMP(IOT_LOG_INFO, "w5500_modbus success.\r\n");

    w5500_spi_set_prescaler(SPI_BAUDRATEPRESCALER_8);
    
    int ret = w5500_modbus_start(&m_cfg);
    DEBUG_DUMP(IOT_LOG_INFO, "w5500_modbus_start returned: %d\r\n", ret);
}
