#ifndef I2C_HUB_MEM_H
#define I2C_HUB_MEM_H
#pragma once

#include "i2c_hub.h"
#include "iot.h"
#include <stdint.h>

#define HUB_SDRAM_BASE SDRAM_START_ADDRESS
#define HUB_SDRAM_SIZE HW_SDRAM_SIZE  /* 32 MB */

#ifndef HUB_I2C_RX_BYTES_DEFAULT
#define HUB_I2C_RX_BYTES_DEFAULT (4U * 1024U * 1024U)  /* 2MB */
#endif
#ifndef HUB_I2C_TX_BYTES_DEFAULT
#define HUB_I2C_TX_BYTES_DEFAULT (4U * 1024U * 1024U)  /* 6MB */
#endif

#define HUB_MEM_CMD_RESET  (0x01u)
#define HUB_MEM_CMD_STATS  (0x02u)

#define HUB_DMA_ALIGN   32U
#define HUB_ALIGN_UP(x)   (((x) + (HUB_DMA_ALIGN-1)) & ~(HUB_DMA_ALIGN-1))
#define HUB_ALIGN_DOWN(x) ((x) & ~(HUB_DMA_ALIGN-1))

#define HUB_ERR_TAG 0x4552524FUL /* 'ERRO' */

/* --------- DCache helpers (align 32B) --------- */
static inline void dcache_invalidate32_range(void *addr, size_t len)
{
    uintptr_t start = (uintptr_t)addr & ~((uintptr_t)(HUB_DMA_ALIGN-1));
    uintptr_t end   = ((uintptr_t)addr + len + (HUB_DMA_ALIGN-1)) & ~((uintptr_t)(HUB_DMA_ALIGN-1));
    SCB_InvalidateDCache_by_Addr((uint32_t *)start, end - start);
}
static inline void dcache_clean32_range(const void *addr, size_t len)
{
    uintptr_t start = (uintptr_t)addr & ~((uintptr_t)(HUB_DMA_ALIGN-1));
    uintptr_t end   = ((uintptr_t)addr + len + (HUB_DMA_ALIGN-1)) & ~((uintptr_t)(HUB_DMA_ALIGN-1));
    SCB_CleanDCache_by_Addr((uint32_t *)start, end - start);
}

/* --------- SPSC ring arena --------- */
/* one producer（use head）、one consumer（use tail） */
typedef struct {
    uint8_t  *base;     /* physical addr */
    uint32_t  size;     /* 2U size */
    volatile uint32_t head;   /* producer write offset */
    volatile uint32_t tail;   /* consumer read offset */
} hub_spsc_arena_t;


extern hub_spsc_arena_t g_hub_rx_arena;   /* Producer: I2C ISR, Consumer: worker */
extern hub_spsc_arena_t g_hub_tx_arena;   /* Producer: worker , Consumer: I2C ISR */

#ifdef __cplusplus
extern "C" {
#endif

/* I2C SPSC helpers */
void hub_spsc_init(hub_spsc_arena_t *a, uintptr_t base_addr, uint32_t bytes);
void *hub_spsc_alloc(hub_spsc_arena_t *a, uint32_t n, uint32_t guard);
void  hub_spsc_free (hub_spsc_arena_t *a, void *p, uint32_t n);
uint32_t hub_spsc_free_space(const hub_spsc_arena_t *a);
uint32_t hub_spsc_mark_head(const hub_spsc_arena_t *a);
void hub_spsc_undo_alloc_to(hub_spsc_arena_t *a, uint32_t mark);

static inline void *hub_sdram_alloc_rx(uint32_t n) { return hub_spsc_alloc(&g_hub_rx_arena, HUB_ALIGN_UP(n), 0U); }
static inline void  hub_sdram_free_rx (void *p, uint32_t n){ hub_spsc_free (&g_hub_rx_arena, p, HUB_ALIGN_UP(n)); }
static inline void *hub_sdram_alloc_tx(uint32_t n) { return hub_spsc_alloc(&g_hub_tx_arena, HUB_ALIGN_UP(n), HUB_DMA_ALIGN); }
static inline void  hub_sdram_free_tx (void *p, uint32_t n){ hub_spsc_free (&g_hub_tx_arena, p, HUB_ALIGN_UP(n)); }

void  *hub_heap_alloc(size_t n); 
void  *hub_heap_alloc_aligned(size_t n, uint32_t align);
void  *hub_heap_calloc(size_t cnt, size_t elemsz);
void   hub_heap_free(void *p);
size_t hub_heap_free_bytes(void);
size_t hub_heap_largest_free_block(void);

static inline void *hub_tmp_alloc(size_t n) { return hub_heap_alloc(n); }
static inline void  hub_tmp_free (void *p)   { hub_heap_free(p); }
void hub_mem_init();
#ifdef __cplusplus
}
#endif
#endif /* I2C_HUB_MEM_H */
