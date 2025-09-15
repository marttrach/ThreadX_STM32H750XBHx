#include "i2c_hub_modbus_server.h"
#include "i2c_hub_w55k.h"

// WIZnet ioLibrary
#include "wizchip_conf.h"
#include "socket.h"
#include "i2c_hub.h"
#include "spi.h"

#ifndef TX_AFFINITY_ENABLE
#define TX_AFFINITY_ENABLE 0
#endif

#ifndef MODBUS_BUF_SZ
#define MODBUS_BUF_SZ 1460   /* one MSS */
#endif

static TX_THREAD s_modbus_thread;
static UCHAR *s_modbus_stack = NULL;
static uint8_t *g_buf_c2u = NULL;
static uint8_t *g_buf_u2c = NULL;
static uint32_t g_buf_sz  = 0;
static w5500_modbus_cfg_t s_cfg;

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

// static int wait_sendok_or_timeout(uint8_t sn, UINT timeout_ms)
// {
//     UINT waited = 0;
//     while (1) {
//         uint8_t ir = getSn_IR(sn);
//         if (ir & Sn_IR_SENDOK) { setSn_IR(sn, Sn_IR_SENDOK); return 0; }
//         if (ir & Sn_IR_TIMEOUT){ setSn_IR(sn, Sn_IR_TIMEOUT); return -1; }
//         uint8_t sr = getSn_SR(sn);
//         if (sr != SOCK_ESTABLISHED && sr != SOCK_CLOSE_WAIT) return -2;
//         if (waited >= timeout_ms) return -3;
//         tx_thread_sleep(MS_TO_TICKS(1));
//         waited += 1;
//     }
// }

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

static void echo_loop(uint8_t sn_cli)
{
    if (!g_buf_c2u || g_buf_sz == 0) return;
    DEBUG_DUMP(IOT_LOG_DEBUG, "echo_loop: start, sn=%d\r\n", sn_cli);

    setSn_KPALVTR(sn_cli, 2);

    while (1) {
        uint8_t sr = getSn_SR(sn_cli);

        if (sr == SOCK_ESTABLISHED) {
            uint8_t ir = getSn_IR(sn_cli);
            if (ir & Sn_IR_CON) setSn_IR(sn_cli, Sn_IR_CON);
        }
        else if (sr == SOCK_CLOSE_WAIT) {
            uint16_t rsr = getSn_RX_RSR(sn_cli);
            if (rsr) {
                if (rsr > g_buf_sz) rsr = g_buf_sz;
                int32_t r = recv(sn_cli, g_buf_c2u, rsr);
                if (r > 0) (void)tcp_send_all_relaxed(sn_cli, g_buf_c2u, (uint16_t)r, 5000);
            }
            disconnect(sn_cli);
            DEBUG_DUMP(IOT_LOG_DEBUG, "echo_loop: CLOSE_WAIT->disconnect, sn=%d\r\n", sn_cli);
            break;
        }
        else {
            DEBUG_DUMP(IOT_LOG_DEBUG, "echo_loop: sr=0x%02X break, sn=%d\r\n", sr, sn_cli);
            break;
        }

        uint16_t rsr = getSn_RX_RSR(sn_cli);
        if (rsr) {
            if (rsr > g_buf_sz) rsr = g_buf_sz;
            int32_t r = recv(sn_cli, g_buf_c2u, rsr);
            if (r <= 0) {
                if (r < 0) DEBUG_DUMP(IOT_LOG_ERR, "echo_loop: recv=%ld err, sn=%d\r\n", (long)r, sn_cli);
                break;
            }
            int rc = tcp_send_all_relaxed(sn_cli, g_buf_c2u, (uint16_t)r, 5000);
            if (rc != 0) {
                DEBUG_DUMP(IOT_LOG_ERR, "echo_loop: tcp_send_all_relaxed rc=%d, sn=%d\r\n", rc, sn_cli);
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
        if (getSn_SR(sn_cli) != SOCK_ESTABLISHED) {
            close_if_open(sn_cli);
            DEBUG_DUMP(IOT_LOG_ERR, "w5500_modbus_thread_entry: listen socket %d not established\r\n", sn_cli);
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
            // DEBUG_DUMP(IOT_LOG_DEBUG, "w5500_modbus_thread_entry: Closing client socket %d\r\n", sn_cli);
        }
        // DEBUG_DUMP(IOT_LOG_DEBUG, "w5500_modbus_thread_entry: Closing client socket %d\r\n", sn_cli);
        close_if_open(sn_cli);
    }
}

int w5500_modbus_start(const w5500_modbus_cfg_t *cfg)
{

    if (!cfg) return -1;
    s_cfg = *cfg;
    DEBUG_DUMP(IOT_LOG_DEBUG, "w5500_modbus_start: listen_socket=%d, upstream_socket=%d, listen_port=%d, bridge_mode=%d\r\n",
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
        DEBUG_DUMP(IOT_LOG_ERR, "tx_thread_create failed, rc=%u (size=%lu)\r\n", rc, (unsigned long)stksz);
        return -2;
    }
    return 0;
}

void w5500_modbus_server_helper(){
    w5500_port_cfg_t pcfg = {
        .hspi     = &hspi1,
        .cs_port  = GPIOG, .cs_pin  = GPIO_PIN_10, /* G10 */
        .rst_port = GPIOH, .rst_pin = GPIO_PIN_4, /* H4 */
        .int_port = NULL,   .int_pin = 0, /* D11 in while loop maybe no need */ 
        // .int_port = GPIOD,   .int_pin = GPIO_PIN_11, /* D11*/
        .mutex_inherit = TX_NO_INHERIT,
    };
    if (w5500_port_init(&pcfg) != 0) {
        return;
    }
    /* no dhcp */
    w5500_net_cfg_t ncfg = {
        .mac = { W5500_DEF_MAC0 },
        .ip  = { W5500_DEF_IP },
        .mask= { W5500_DEF_MASK },
        .gw  = { W5500_DEF_GW },
        .dns = { W5500_DEF_DNS },
        .use_dhcp = 0,
    };
    DEBUG_DUMP(IOT_LOG_DEBUG, "w5500_modbus\r\n");
    if (w5500_bringup(&ncfg) != 0) {
        /* plug */
        return;
    }
    DEBUG_DUMP(IOT_LOG_DEBUG, "w5500_modbus success\r\n");

    // w5500_spi_set_prescaler(SPI_BAUDRATEPRESCALER_8);

    w5500_modbus_cfg_t mcfg = {
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
    int ret = w5500_modbus_start(&mcfg);
    DEBUG_DUMP(IOT_LOG_DEBUG, "w5500_modbus_start returned %d\r\n", ret);
}
