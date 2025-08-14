#include "i2c_hub_mem.h"

hub_spsc_arena_t g_hub_rx_arena;  // Producer: I2C2, Consumer: worker
hub_spsc_arena_t g_hub_tx_arena;  // Producer: worker , Consumer: I2C2
