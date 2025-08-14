#ifndef I2C_HUB_MEM_H
#define I2C_HUB_MEM_H
#pragma once

#include "i2c_hub.h"
#include <stdint.h>

#define HUB_SDRAM_BASE 0xC0000000UL
#define HUB_SDRAM_SIZE 0x02000000UL  /* 32 MB */

#define HUB_DMA_ALIGN   32U
#define HUB_ALIGN_UP(x) (((x) + (HUB_DMA_ALIGN-1)) & ~(HUB_DMA_ALIGN-1))

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


/* init /size (maybe rx and tx can seperate) */
static inline void hub_spsc_init(hub_spsc_arena_t *a, uintptr_t base_addr, uint32_t bytes)
{
    a->base = (uint8_t *)base_addr;
    a->size = bytes;
    a->head = 0;
    a->tail = 0;
}

static inline const char* __arena_name(const hub_spsc_arena_t *a) {
    return (a == &g_hub_rx_arena) ? "RX" : (a == &g_hub_tx_arena) ? "TX" : "?";
}

static inline void *hub_spsc_alloc(hub_spsc_arena_t *a, uint32_t n, uint32_t guard)
{
    n = HUB_ALIGN_UP(n);

    uint32_t head = a->head;
    uint32_t tail = a->tail;
    uint32_t size = a->size;
    if (head >= tail) {
        uint32_t free_end = size - head;
        uint32_t max_end  = (free_end > guard) ? (free_end - guard) : 0;

        if (n <= max_end) {
            a->head = head + n;
            return a->base + head;
        }
        /* wrap */
        if (tail >= (n + guard)) {
            a->head = n;
            return a->base;
        }
        DEBUG_DUMP(IOT_LOG_ERR, "[%s] alloc fail (wrap fail). n=%lu, free_end=%lu, tail=%lu\r\n",
                   __arena_name(a), (unsigned long)n, (unsigned long)free_end, (unsigned long)tail);
        return NULL;

    } else {
        uint32_t free_between = tail - head;
        uint32_t max_between  = (free_between > guard) ? (free_between - guard) : 0;

        if (n <= max_between) {
            a->head = head + n;
            return a->base + head;
        }
        DEBUG_DUMP(IOT_LOG_ERR, "[%s] alloc fail. n=%lu, free_between=%lu, max_between=%lu\r\n",
                   __arena_name(a), (unsigned long)n, (unsigned long)free_between, (unsigned long)max_between);
        return NULL;
    }
}

/* let alloc fifo */
static inline void hub_spsc_free(hub_spsc_arena_t *a, void *p, uint32_t n)
{
    n = HUB_ALIGN_UP(n);

#if 1
    uint8_t *exp = a->base + a->tail;
    if (exp >= a->base + a->size) exp -= a->size;
    if ((uint8_t*)p != exp) {
        DEBUG_DUMP(IOT_LOG_ERR, "[%s] FREE pointer mismatch: p=%p exp=%p, n=%lu (tail=%lu, head=%lu)\r\n",
                   __arena_name(a), p, exp, (unsigned long)n, (unsigned long)a->tail, (unsigned long)a->head);
        return;
    }
#endif

    __DMB();
    uint32_t t = a->tail + n;
    if (t >= a->size) t -= a->size;
    a->tail = t;
}

/* debug/observe , calculate last*/
static inline uint32_t hub_spsc_free_space(const hub_spsc_arena_t *a)
{
    uint32_t head = a->head, tail = a->tail, size = a->size;
    if (head >= tail) return (size - (head - tail));
    else              return (tail - head);
}

static inline void  hub_mem_init(void)
{
    /* half half*/
    const uint32_t half = HUB_SDRAM_SIZE / 2U;
    hub_spsc_init(&g_hub_rx_arena, HUB_SDRAM_BASE,          half);
    hub_spsc_init(&g_hub_tx_arena, HUB_SDRAM_BASE + half,   half);
}

static inline uint32_t hub_spsc_mark_head(const hub_spsc_arena_t *a)
{
    return a->head;
}

/* rollback ptr */
static inline void hub_spsc_undo_alloc_to(hub_spsc_arena_t *a, uint32_t mark)
{
    a->head = mark;
    __DMB();
}


static inline void *hub_sdram_alloc_rx(uint32_t n) { return hub_spsc_alloc(&g_hub_rx_arena, n, 0U); }
static inline void  hub_sdram_free_rx (void *p, uint32_t n){ hub_spsc_free(&g_hub_rx_arena, p, n); }
static inline void *hub_sdram_alloc_tx(uint32_t n) { return hub_spsc_alloc(&g_hub_tx_arena, n, HUB_DMA_ALIGN); }
static inline void  hub_sdram_free_tx (void *p, uint32_t n){ hub_spsc_free(&g_hub_tx_arena, p, n); }

#endif /* I2C_HUB_MEM_H */
