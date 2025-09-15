#include "i2c_hub_mem.h"

hub_spsc_arena_t g_hub_rx_arena;  // Producer: I2C2, Consumer: worker
hub_spsc_arena_t g_hub_tx_arena;  // Producer: worker , Consumer: I2C2

void hub_spsc_init(hub_spsc_arena_t *a, uintptr_t base_addr, uint32_t bytes)
{
    a->base = (uint8_t *)base_addr;
    a->size = HUB_ALIGN_UP(bytes);
    a->head = 0;
    a->tail = 0;
}

static inline const char* __arena_name(const hub_spsc_arena_t *a) {
    return (a == &g_hub_rx_arena) ? "RX" : (a == &g_hub_tx_arena) ? "TX" : "?";
}

static inline uint32_t hub_mod_u32(uint32_t off, uint32_t size)
{
    if (off >= size) off -= size;
    if (off >= size) off %= size;
    return off;
}

void *hub_spsc_alloc(hub_spsc_arena_t *a, uint32_t n, uint32_t guard)
{
    n     = HUB_ALIGN_UP(n);
    guard = HUB_ALIGN_UP(guard);

    uint32_t head = a->head;
    uint32_t tail = a->tail;
    const uint32_t size = a->size;

    if (n > size) {
        DEBUG_DUMP(IOT_LOG_ERR, "hub_spsc_alloc(%s): req too big n=%lu > size=%lu\r\n",
                   __arena_name(a), (unsigned long)n, (unsigned long)size);
        return NULL;
    }

    if (head < tail) {
        uint32_t space = tail - head;
        if (n + guard <= space) {
            uint8_t *p = a->base + head;
            a->head = head + n;
            __DMB();
            return p;
        }
        DEBUG_DUMP(IOT_LOG_ERR, "hub_spsc_alloc(%s): head<tail no space (req=%lu, guard=%lu, head=%lu, tail=%lu, size=%lu)\r\n",
                   __arena_name(a),(unsigned long)n,(unsigned long)guard,(unsigned long)head,(unsigned long)tail,(unsigned long)size);
        return NULL;
    }

    uint32_t end_space = size - head;
    if (n + guard <= end_space) {
        uint8_t *p = a->base + head;
        head += n; if (head >= size) head -= size;
        a->head = head;
        __DMB();
        return p;
    }

    if (n + guard <= tail) {
        uint8_t *p = a->base;
        a->head = n;
        __DMB();
        return p;
    }

    DEBUG_DUMP(IOT_LOG_ERR, "hub_spsc_alloc(%s): head>=tail no space (req=%lu, guard=%lu, head=%lu, tail=%lu, size=%lu)\r\n",
               __arena_name(a),(unsigned long)n,(unsigned long)guard,(unsigned long)head,(unsigned long)tail,(unsigned long)size);
    return NULL;
}

void hub_spsc_free(hub_spsc_arena_t *a, void *p, uint32_t n)
{
    n = HUB_ALIGN_UP(n);

    uint32_t tail0 = a->tail;
    uint32_t tail  = hub_mod_u32(tail0, a->size);
    uint8_t *exp   = a->base + tail;

    if ((uint8_t*)p != exp) {
        uint32_t p_off   = (uint32_t)((uint8_t*)p - a->base);
        uint32_t exp_off = tail;
        uint32_t head    = hub_mod_u32(a->head, a->size);

        DEBUG_DUMP(IOT_LOG_ERR,
          "hub_spsc_free(%s): out-of-order free p=%p(off=%lu) expected=%p(off=%lu) "
          "(head=%lu, tail=%lu(raw), size=%lu, base=%p)\r\n",
          __arena_name(a), p, (unsigned long)p_off, exp, (unsigned long)exp_off,
          (unsigned long)head, (unsigned long)tail0, (unsigned long)a->size, a->base);
        return;
    }

    __DMB();
    uint32_t t = tail + n;
    t = hub_mod_u32(t, a->size);
    a->tail = t;
    __DMB();
}

uint32_t hub_spsc_free_space(const hub_spsc_arena_t *a)
{
    uint32_t head = hub_mod_u32(a->head, a->size);
    uint32_t tail = hub_mod_u32(a->tail, a->size);
    uint32_t size = a->size;
    if (head >= tail) return size - (head - tail);
    else              return tail - head;
}

uint32_t hub_spsc_mark_head(const hub_spsc_arena_t *a) { return a->head; }
void hub_spsc_undo_alloc_to(hub_spsc_arena_t *a, uint32_t mark) { a->head = mark; __DMB(); }


typedef union hub_block_u hub_block_t;
union __attribute__((aligned(32))) hub_block_u {
    struct {
        uint32_t size;
        hub_block_t *next;
        uint8_t _pad[32 - 8];
    } freeb;
    struct {
        uint32_t size;
        uint32_t magic;
        uint8_t  rsv[32 - 8];
    } alloc;
};

#define HUB_HEAP_MAGIC  (0xC0DEC0DEu)
#define HDR_SZ          ((uint32_t)sizeof(hub_block_t))
#define MIN_BLK_SZ      ((uint32_t)32)

typedef struct {
    uint8_t    *base;
    uint32_t    size;
    hub_block_t *free_list;
    TX_MUTEX    mtx;
} hub_heap_t;

static hub_heap_t g_heap = {0};

static void heap_lock(void)   { tx_mutex_get(&g_heap.mtx, TX_WAIT_FOREVER); }
static void heap_unlock(void) { tx_mutex_put(&g_heap.mtx); }

static void heap_init_region(void *base, uint32_t bytes)
{
    g_heap.base = (uint8_t*)HUB_ALIGN_UP((uintptr_t)base);
    uint32_t sz = bytes - (uint32_t)((uintptr_t)g_heap.base - (uintptr_t)base);
    sz = HUB_ALIGN_DOWN(sz);
    g_heap.size = sz;

    hub_block_t *b = (hub_block_t*)g_heap.base;
    b->freeb.size = sz;
    b->freeb.next = NULL;
    g_heap.free_list = b;
}

static void free_list_insert(hub_block_t *b)
{
    hub_block_t *prev = NULL, *cur = g_heap.free_list;

    while (cur && (uintptr_t)cur < (uintptr_t)b) {
        prev = cur;
        cur  = cur->freeb.next;
    }

    if (prev) prev->freeb.next = b; else g_heap.free_list = b;
    b->freeb.next = cur;

    if (cur && ((uint8_t*)b + b->freeb.size == (uint8_t*)cur)) {
        b->freeb.size += cur->freeb.size;
        b->freeb.next  = cur->freeb.next;
    }
    if (prev && ((uint8_t*)prev + prev->freeb.size == (uint8_t*)b)) {
        prev->freeb.size += b->freeb.size;
        prev->freeb.next  = b->freeb.next;
    }
}

static hub_block_t *split_block(hub_block_t *b, uint32_t need)
{
    if (b->freeb.size >= need + MIN_BLK_SZ) {
        hub_block_t *remain = (hub_block_t*)((uint8_t*)b + need);
        remain->freeb.size = b->freeb.size - need;
        remain->freeb.next = b->freeb.next;
        b->freeb.size = need;
        return remain;
    }
    return b->freeb.next;
}

void *hub_heap_alloc_aligned(size_t n, uint32_t align)
{
    if (align < HUB_DMA_ALIGN) align = HUB_DMA_ALIGN;
    uint32_t need = (uint32_t)n;
    if (need == 0) need = HUB_DMA_ALIGN;
    uint32_t total = HUB_ALIGN_UP(HDR_SZ + need);
    if (total < (HDR_SZ + MIN_BLK_SZ)) total = HDR_SZ + MIN_BLK_SZ;
    total = HUB_ALIGN_UP(total);

    heap_lock();

    hub_block_t *prev = NULL, *cur = g_heap.free_list;
    while (cur) {
        uintptr_t cur_addr = (uintptr_t)cur;
        uintptr_t payload  = HUB_ALIGN_UP(cur_addr + HDR_SZ);
        uint32_t front_gap = (uint32_t)(payload - (cur_addr + HDR_SZ));
        uint32_t need2 = HUB_ALIGN_UP(HDR_SZ + front_gap + need);
        if (need2 > total) total = need2;

        if (cur->freeb.size >= total) {
            if (prev) prev->freeb.next = split_block(cur, total);
            else      g_heap.free_list = split_block(cur, total);

            cur->alloc.size  = total;
            cur->alloc.magic = HUB_HEAP_MAGIC;

            void *ret = (void*)((uint8_t*)cur + HDR_SZ);
            heap_unlock();
            return ret;
        }
        prev = cur;
        cur  = cur->freeb.next;
    }

    heap_unlock();
    return NULL;
}

void *hub_heap_alloc(size_t n) { return hub_heap_alloc_aligned(n, HUB_DMA_ALIGN); }

void *hub_heap_calloc(size_t cnt, size_t elemsz)
{
    size_t n = cnt * elemsz;
    void *p = hub_heap_alloc(n);
    if (p) memset(p, 0, n);
    return p;
}

void hub_heap_free(void *p)
{
    if (!p) return;
    hub_block_t *b = (hub_block_t*)((uint8_t*)p - HDR_SZ);
    if (b->alloc.magic != HUB_HEAP_MAGIC) return;
    b->alloc.magic = 0;

    heap_lock();
    b->freeb.next = NULL;
    free_list_insert(b);
    heap_unlock();
}

size_t hub_heap_free_bytes(void)
{
    heap_lock();
    size_t sum = 0;
    for (hub_block_t *c = g_heap.free_list; c; c = c->freeb.next) sum += c->freeb.size;
    heap_unlock();
    return sum;
}
size_t hub_heap_largest_free_block(void)
{
    heap_lock();
    size_t maxb = 0;
    for (hub_block_t *c = g_heap.free_list; c; c = c->freeb.next)
        if (c->freeb.size > maxb) maxb = c->freeb.size;
    heap_unlock();
    return maxb;
}

static uint32_t g_rx_bytes_cfg = HUB_I2C_RX_BYTES_DEFAULT;
static uint32_t g_tx_bytes_cfg = HUB_I2C_TX_BYTES_DEFAULT;

// void hub_mem_config_i2c(uint32_t rx_bytes, uint32_t tx_bytes)
// {
//     g_rx_bytes_cfg = HUB_ALIGN_UP(rx_bytes);
//     g_tx_bytes_cfg = HUB_ALIGN_UP(tx_bytes);
//     if (g_rx_bytes_cfg + g_tx_bytes_cfg > HUB_SDRAM_SIZE) {
//         g_rx_bytes_cfg = HUB_I2C_RX_BYTES_DEFAULT;
//         g_tx_bytes_cfg = HUB_I2C_TX_BYTES_DEFAULT;
//     }
// }

void hub_mem_init(void)
{
    uintptr_t rx_base = HUB_SDRAM_BASE;
    uintptr_t tx_base = rx_base + g_rx_bytes_cfg;
    uintptr_t heap_base = tx_base + g_tx_bytes_cfg;

    hub_spsc_init(&g_hub_rx_arena, rx_base,  g_rx_bytes_cfg);
    hub_spsc_init(&g_hub_tx_arena, tx_base,  g_tx_bytes_cfg);

    uint32_t heap_bytes = (uint32_t)(HUB_SDRAM_SIZE - (heap_base - HUB_SDRAM_BASE));
    heap_init_region((void*)heap_base, heap_bytes);

    static UINT mtx_inited = 0;
    if (!mtx_inited) {
        tx_mutex_create(&g_heap.mtx, "hub_heap_mtx", TX_INHERIT);
        mtx_inited = 1;
    }
}
