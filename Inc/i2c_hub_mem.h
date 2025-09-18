#ifndef I2C_HUB_MEM_H
#define I2C_HUB_MEM_H
#pragma once

#include "i2c_hub.h"
#include "iot.h"
#include <stdint.h>

#define HUB_SDRAM_BASE SDRAM_START_ADDRESS
#define HUB_SDRAM_SIZE HW_SDRAM_SIZE  /* 32 MB */

#ifndef HUB_RX_GUARD
#if TEST_SMALL_SDRAM
#define HUB_RX_GUARD (64U) /* test 64B */
#else
#define HUB_RX_GUARD (128U*1024U) /* 128KB */
#endif
#endif
#ifndef HUB_TX_GUARD
#if TEST_SMALL_SDRAM
#define HUB_TX_GUARD (64U) /* test 64B */
#else
#define HUB_TX_GUARD (128U*1024U) /* 128KB */
#endif
#endif

#ifndef HUB_I2C_RX_BYTES_DEFAULT
#if TEST_SMALL_SDRAM
#define HUB_I2C_RX_BYTES_DEFAULT (4U * 1024U)  /* test 4K */
#else
#define HUB_I2C_RX_BYTES_DEFAULT (4U * 1024U * 1024U)  /* 4MB */
#endif
#endif
#ifndef HUB_I2C_TX_BYTES_DEFAULT
#if TEST_SMALL_SDRAM
#define HUB_I2C_TX_BYTES_DEFAULT (4U * 1024U)  /* test 4K */
#else
#define HUB_I2C_TX_BYTES_DEFAULT (4U * 1024U * 1024U)  /* 4MB */
#endif
#endif

#ifndef HUB_I2C_RX_MAX_FRAME
#define HUB_I2C_RX_MAX_FRAME  HUB_I2C_RX_BYTES_DEFAULT/4U
#endif
#ifndef HUB_I2C_TX_MAX_FRAME
#define HUB_I2C_TX_MAX_FRAME  HUB_I2C_TX_BYTES_DEFAULT/4U
#endif

#define HUB_MEM_CMD_RESET  (0x01u)
#define HUB_MEM_CMD_STATS  (0x02u)

#define HUB_DMA_ALIGN   32U
#define HUB_ALIGN_UP(x)   (((x) + (HUB_DMA_ALIGN-1)) & ~(HUB_DMA_ALIGN-1))
#define HUB_ALIGN_DOWN(x) ((x) & ~(HUB_DMA_ALIGN-1))

#define HUB_ERR_TAG 0x4552524FUL /* 'ERRO' */
#define HUB_BLK_MAGIC   (0x48554252UL)      /* 'HUBR' */
#define HUB_DONE_MAGIC  (0xD02E0001UL)
#define HUB_HDR_SIZE    (sizeof(hub_blk_hdr_t))   /* 32B */

/* --------- DCache helpers (align 32B) --------- */
static inline void dcache_invalidate32_range(const void *addr, size_t len)
{
    if (!len) return;
    uintptr_t start = (uintptr_t)addr & ~(uintptr_t)31;
    uintptr_t end   = ((uintptr_t)addr + len + 31u) & ~(uintptr_t)31;
    SCB_InvalidateDCache_by_Addr((void*)start, (int32_t)(end - start));
    __DSB(); __ISB();
}

static inline void dcache_clean32_range(const void *addr, size_t len)
{
    if (!len) return;
    uintptr_t start = (uintptr_t)addr & ~(uintptr_t)31;
    uintptr_t end   = ((uintptr_t)addr + len + 31u) & ~(uintptr_t)31;
    SCB_CleanDCache_by_Addr((void*)start, (int32_t)(end - start));
    __DSB(); __ISB();
}
/* --------- SPSC ring arena --------- */
/* one producer（use head）、one consumer（use tail） */
typedef struct {
    uint8_t  *base;     /* physical addr */
    uint32_t  size;     /* 2U size */
    uint32_t head;   /* producer write offset */
    uint32_t tail;   /* consumer read offset */
} hub_spsc_arena_t;

typedef struct {
    uint32_t len_aligned;      /* allocation (payload_len + padding) aligned length */
    uint32_t seq;              /* sequential can find error */
    volatile uint32_t done;    /* 0=Not Done；0xD0NE0001=Done (reclaimable) */
    uint32_t rsv0;             /* reserved 0 , maybe can use flag */
    uint32_t magic;            /* 0x48554252 'HUBR'：mem break */
    uint32_t rsv1;             /* reserved 1 */
    uint32_t rsv2;             /* reserved 2 */
    uint32_t rsv3;             /* reserved 3 */
} __attribute__((packed,aligned(32))) hub_blk_hdr_t;

extern hub_spsc_arena_t g_hub_rx_arena;   /* Producer: I2C ISR, Consumer: worker */
extern hub_spsc_arena_t g_hub_tx_arena;   /* Producer: worker , Consumer: I2C ISR */

#ifdef __cplusplus
extern "C" {
#endif

/* I2C SPSC helpers */
void hub_spsc_init (hub_spsc_arena_t *a, uintptr_t base_addr, uint32_t bytes);
void *hub_spsc_alloc_frame(hub_spsc_arena_t *a, uint32_t payload_len, uint32_t guard);
void  hub_spsc_done       (hub_spsc_arena_t *a, void *payload_ptr);
uint32_t hub_spsc_free_space(const hub_spsc_arena_t *a);
uint32_t hub_spsc_mark_head(const hub_spsc_arena_t *a);
void hub_spsc_undo_alloc_to(hub_spsc_arena_t *a, uint32_t mark);

void  hub_spsc_reclaim_fast(hub_spsc_arena_t *a, uint32_t need_plus_guard, uint32_t budget_chunks);
void  hub_spsc_reclaim_all(hub_spsc_arena_t *a);
void  hub_spsc_reclaim_for_alloc(hub_spsc_arena_t *a, uint32_t need, uint32_t guard, uint32_t budget_chunks);
void  hub_spsc_reset      (hub_spsc_arena_t *a);

static inline void *hub_sdram_alloc_rx(uint32_t n) { return hub_spsc_alloc_frame(&g_hub_rx_arena, HUB_ALIGN_UP(n), HUB_RX_GUARD); }
static inline void  hub_sdram_free_rx (void *p, uint32_t n){ hub_spsc_done(&g_hub_rx_arena, p); }
static inline void *hub_sdram_alloc_tx(uint32_t n) { return hub_spsc_alloc_frame(&g_hub_tx_arena, HUB_ALIGN_UP(n), HUB_TX_GUARD); }
static inline void  hub_sdram_free_tx (void *p, uint32_t n){ hub_spsc_done(&g_hub_tx_arena, p); }

void  *hub_heap_alloc(size_t n); 
void  *hub_heap_alloc_aligned(size_t n, uint32_t align);
#if 0
void  *hub_heap_calloc(size_t cnt, size_t elemsz);
#endif
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
