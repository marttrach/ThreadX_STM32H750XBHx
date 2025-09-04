#include "i2c_hub_w55k.h"

// WIZnet ioLibrary
#include "wizchip_conf.h"
#include "socket.h"
#include "w5500.h"

#ifndef TX_TIMER_TICKS_PER_SECOND
#define TX_TIMER_TICKS_PER_SECOND 100
#endif

#define PHYCFGR_LNK          (1u << 0)     // Link status (1: up)
#define PHYCFGR_SPD          (1u << 1)     // 1: 100M, 0: 10M
#define PHYCFGR_DPX          (1u << 2)     // 1: Full, 0: Half
#define PHYCFGR_OPMDC_MASK   (0x7u << 3)   // [5:3]
#define PHYCFGR_OPMD         (1u << 6)     // 1: Software config, 0: HW strap
#define PHYCFGR_RST          (1u << 7)     // 1: Reset

//w5500 OPMDC=110 means "All capable, Auto-Negotiation"
#define PHYCFGR_OPMDC_ALL_CAP_AUTONEGO   (0x6u << 3)

static w5500_port_cfg_t g_pcfg = {0};
static TX_MUTEX g_w5500_mutex;
static uint8_t g_mutex_ready = 0;

static inline void cs_select(void)   { HAL_GPIO_WritePin(g_pcfg.cs_port, g_pcfg.cs_pin, GPIO_PIN_RESET); }
static inline void cs_deselect(void) { HAL_GPIO_WritePin(g_pcfg.cs_port, g_pcfg.cs_pin, GPIO_PIN_SET);   }

static void cb_cs_select(void)   { if (g_mutex_ready) tx_mutex_get(&g_w5500_mutex, TX_WAIT_FOREVER); cs_select(); }
static void cb_cs_deselect(void) { cs_deselect(); if (g_mutex_ready) tx_mutex_put(&g_w5500_mutex); }

static uint8_t cb_spi_rb(void) {
    uint8_t tx=0xFF, rx=0;
    HAL_SPI_TransmitReceive(g_pcfg.hspi, &tx, &rx, 1, W5500_SPI_TIMEOUT_MS);
    return rx;
}

static void cb_spi_wb(uint8_t b) {
    HAL_SPI_Transmit(g_pcfg.hspi, &b, 1, W5500_SPI_TIMEOUT_MS);
}

static void cb_spi_wburst(uint8_t *pbuf, uint16_t len) {
    if (len) HAL_SPI_Transmit(g_pcfg.hspi, pbuf, len, W5500_SPI_TIMEOUT_MS);
}

static void cb_spi_rburst(uint8_t *pbuf, uint16_t len) {
    if (len) HAL_SPI_Receive(g_pcfg.hspi, pbuf, len, W5500_SPI_TIMEOUT_MS);
}

void w5500_lock(void)   { if (g_mutex_ready) tx_mutex_get(&g_w5500_mutex, TX_WAIT_FOREVER); }
void w5500_unlock(void) { if (g_mutex_ready) tx_mutex_put(&g_w5500_mutex); }

void w5500_delay_ms(uint32_t ms) { tx_thread_sleep(MS_TO_TICKS(ms)); }

void w5500_hw_reset(void)
{
    if (!g_pcfg.rst_port) return;
    HAL_GPIO_WritePin(g_pcfg.rst_port, g_pcfg.rst_pin, GPIO_PIN_RESET);
    w5500_delay_ms(2);
    HAL_GPIO_WritePin(g_pcfg.rst_port, g_pcfg.rst_pin, GPIO_PIN_SET);
    w5500_delay_ms(150);
}

/* set fast pre clock*/
int w5500_spi_set_prescaler(uint32_t prescaler)
{
    if (!g_pcfg.hspi) return -1;
    __HAL_SPI_DISABLE(g_pcfg.hspi);
    g_pcfg.hspi->Init.BaudRatePrescaler = prescaler;
    if (HAL_SPI_Init(g_pcfg.hspi) != HAL_OK) return -2;
    __HAL_SPI_ENABLE(g_pcfg.hspi);
    return 0;
}

/* link status */
int w5500_is_link_up(void)
{
    uint8_t link=0;
    ctlwizchip(CW_GET_PHYLINK, (void*)&link);
    return (link == PHY_LINK_ON) ? 1 : 0;
}

int w5500_port_init(const w5500_port_cfg_t *pcfg)
{
    if (!pcfg || !pcfg->hspi || !pcfg->cs_port || !pcfg->cs_pin) return -1;
    g_pcfg = *pcfg;

    HAL_GPIO_WritePin(g_pcfg.cs_port, g_pcfg.cs_pin, GPIO_PIN_SET);
    w5500_hw_reset();

    // mutex (multi threadx cause deadlock)
    if (tx_mutex_create(&g_w5500_mutex, "w5500_mutex",
                        pcfg->mutex_inherit ? TX_INHERIT : TX_NO_INHERIT) != TX_SUCCESS)
        return -2;
    g_mutex_ready = 1;

    // register w55k lib
    reg_wizchip_cs_cbfunc(cb_cs_select, cb_cs_deselect);
    reg_wizchip_spi_cbfunc(cb_spi_rb, cb_spi_wb);
    reg_wizchip_spiburst_cbfunc(cb_spi_rburst, cb_spi_wburst);
    return 0;
}

static void w5500_phy_dump(const char *tag)
{
    char chip_id[8] = {0};
    ctlwizchip(CW_GET_ID, (void*)chip_id); /* w5500 */

    uint8_t phy = getPHYCFGR();
    int link   = (phy & 0x01) ? 1 : 0;     // LNK
    int speed  = (phy & 0x02) ? 100 : 10;  // SPD (1=100M,0=10M)
    int duplex = (phy & 0x04) ? 1 : 0;     // DPX (1=Full,0=Half)
    int opmd   = (phy & 0x40) ? 1 : 0;     // OPMD (1=soft reg, 0=hard cs)
    uint8_t opmdc = (phy >> 3) & 0x07;     // OPMDC[5:3] worker/save/force mode

    DEBUG_DUMP(IOT_LOG_DEBUG,
      "[%s] chip=%s PHYCFGR=0x%02X link=%d speed=%dM duplex=%s opmd=%s opmdc=%u\r\n",
      tag, chip_id[0] ? chip_id : "?", phy, link, speed, duplex ? "Full":"Half",
      opmd ? "SW":"HW", opmdc);
}

static int w5500_phy_try_autonego(uint32_t timeout_ms)
{
#ifdef CW_SET_PHYCONF
    wiz_PhyConf phyconf;
    phyconf.by     = PHY_CONFBY_SW;        /* soft control*/
    phyconf.mode   = PHY_MODE_AUTONEGO;    /* auto negotiation */
    phyconf.speed  = PHY_SPEED_100;        /* 100M */
    phyconf.duplex = PHY_DUPLEX_FULL;      /* full duplex */
    ctlwizchip(CW_SET_PHYCONF, (void*)&phyconf);
#endif

    uint32_t elapsed = 0;
    while (elapsed < timeout_ms)
    {
        uint8_t link = 0;
        ctlwizchip(CW_GET_PHYLINK, (void*)&link);
        if (link == PHY_LINK_ON) return 0;
        w5500_delay_ms(50);
        elapsed += 50;
    }
    return -1;
}

static void w5500_phy_force_autonego_sw(void)
{
    uint8_t phy = getPHYCFGR();

    // soft control
    phy |= PHYCFGR_OPMD;

    phy = (uint8_t)((phy & ~PHYCFGR_OPMDC_MASK) | PHYCFGR_OPMDC_ALL_CAP_AUTONEGO);

    setPHYCFGR(phy);

    phy = getPHYCFGR();
    setPHYCFGR((uint8_t)(phy | PHYCFGR_RST));
    w5500_delay_ms(2);
    setPHYCFGR((uint8_t)(phy & ~PHYCFGR_RST));
}

int w5500_bringup(const w5500_net_cfg_t *ncfg)
{
    if (!ncfg) return -1;

    uint8_t txsize[8] = {2,2,2,2,2,2,2,2};
    uint8_t rxsize[8] = {2,2,2,2,2,2,2,2};

    if (wizchip_init(txsize, rxsize) != 0) return -2;

    wiz_NetInfo ni;
    ni.mac[0]=ncfg->mac[0]; ni.mac[1]=ncfg->mac[1]; ni.mac[2]=ncfg->mac[2];
    ni.mac[3]=ncfg->mac[3]; ni.mac[4]=ncfg->mac[4]; ni.mac[5]=ncfg->mac[5];
    ni.ip[0]=ncfg->ip[0];   ni.ip[1]=ncfg->ip[1];   ni.ip[2]=ncfg->ip[2];   ni.ip[3]=ncfg->ip[3];
    ni.sn[0]=ncfg->mask[0]; ni.sn[1]=ncfg->mask[1]; ni.sn[2]=ncfg->mask[2]; ni.sn[3]=ncfg->mask[3];
    ni.gw[0]=ncfg->gw[0];   ni.gw[1]=ncfg->gw[1];   ni.gw[2]=ncfg->gw[2];   ni.gw[3]=ncfg->gw[3];
    ni.dns[0]=ncfg->dns[0]; ni.dns[1]=ncfg->dns[1]; ni.dns[2]=ncfg->dns[2]; ni.dns[3]=ncfg->dns[3];

    // DHCP (need to add to the cmakefilelist)
    ni.dhcp = ncfg->use_dhcp ? NETINFO_DHCP : NETINFO_STATIC;

    ctlnetwork(CN_SET_NETINFO, (void*)&ni);
    DEBUG_DUMP(IOT_LOG_DEBUG, "w5500_net_info: mac=%02X:%02X:%02X:%02X:%02X:%02X, "
        "ip=%d.%d.%d.%d, mask=%d.%d.%d.%d, gw=%d.%d.%d.%d, dns=%d.%d.%d.%d\r\n",
        ni.mac[0], ni.mac[1], ni.mac[2], ni.mac[3], ni.mac[4], ni.mac[5],
        ni.ip[0], ni.ip[1], ni.ip[2], ni.ip[3],
        ni.sn[0], ni.sn[1], ni.sn[2], ni.sn[3],
        ni.gw[0], ni.gw[1], ni.gw[2], ni.gw[3],
        ni.dns[0], ni.dns[1], ni.dns[2], ni.dns[3]);
    // wait PHY layer 
    // uint8_t ver = getVERSIONR();  /* w5500 -> 0x04 */
    // DEBUG_DUMP(IOT_LOG_DEBUG, "W5500 VERSIONR=0x%02X\r\n", ver);
    w5500_phy_dump("before_soft");
    // if (!w5500_is_link_up()) {
    //     w5500_phy_force_autonego_sw();  // 7 = full functional auto negotiation
    //     w5500_phy_dump("after_force");
    // }
    (void)w5500_phy_try_autonego(1500);
    w5500_phy_dump("after_soft_phycfg, and before_force");
    // w5500_phy_force_autonego_sw();
    for (int i=0; i<200; ++i) {
        if (w5500_is_link_up()) break;
        w5500_delay_ms(50);
    }
    if (!w5500_is_link_up()) {
        DEBUG_DUMP(IOT_LOG_ERR, "w5500_bringup: PHY link down\r\n");
        return -3;
    }

    return 0;
}
