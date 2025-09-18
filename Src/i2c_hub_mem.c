#include "i2c_hub_mem.h"

#define HUB_MEM_HEAP_MAGIC  (0xC0DEC0DEu)
#define HUB_MEM_HDR_BLK_SZ          ((uint32_t)sizeof(hub_block_t))
#define HUB_MEM_MIN_BLK_SZ      ((uint32_t)32)

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

typedef struct {
    uint8_t    *base;
    uint32_t    size;
    hub_block_t *free_list;
    TX_MUTEX    mtx;
} hub_heap_t;

static hub_heap_t g_heap = {0};

static void heap_lock(void)   { tx_mutex_get(&g_heap.mtx, TX_WAIT_FOREVER); }
static void heap_unlock(void) { tx_mutex_put(&g_heap.mtx); }

hub_spsc_arena_t g_hub_rx_arena;  // Producer: I2C2, Consumer: worker
hub_spsc_arena_t g_hub_tx_arena;  // Producer: worker , Consumer: I2C2

static uint32_t g_rx_seq = 0;
static uint32_t g_tx_seq = 0;

void hub_spsc_init(hub_spsc_arena_t *a, uintptr_t base_addr, uint32_t bytes)
{
    a->base = (uint8_t *)base_addr;
    a->size = HUB_ALIGN_UP(bytes);
    a->head = 0;
    a->tail = 0;

    if (a == &g_hub_rx_arena) g_rx_seq = 0;
    if (a == &g_hub_tx_arena) g_tx_seq = 0;
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

static inline hub_blk_hdr_t* hdr_at(const hub_spsc_arena_t *a, uint32_t off) {
    return (hub_blk_hdr_t *)(void*)(a->base + off);
}

void hub_spsc_reclaim_fast(hub_spsc_arena_t *a, uint32_t need_plus_guard, uint32_t budget_chunks)
{
    uint32_t reclaimed_bytes = 0, cnt = 0;
    for (;;) {
        if (budget_chunks && cnt >= budget_chunks) break;

        uint32_t tail = a->tail;
        uint32_t head = a->head;
        if (tail == head) break;

        hub_blk_hdr_t *h = hdr_at(a, tail);
        __DMB();

        if (h->magic != HUB_BLK_MAGIC) {
            if (head < tail) {
                uint32_t gap = a->size - tail;
                if ((gap & (HUB_DMA_ALIGN - 1U)) == 0U) {
                    a->tail = 0;
                    __DMB();
                    DEBUG_DUMP(IOT_LOG_DEBUG, "hub_spsc_reclaim(%s): wrap-gap detected at off=%lu → tail=0 (self-heal)\r\n",
                               __arena_name(a), (unsigned long)tail);
                    continue;
                }
            }
            DEBUG_DUMP(IOT_LOG_ERR, "hub_spsc_reclaim(%s): BAD MAGIC at off=%lu\r\n",
                       __arena_name(a), (unsigned long)tail);
            break;
        }

        if (h->done != HUB_DONE_MAGIC) break;

        uint32_t n = h->len_aligned + HUB_HDR_SIZE;
        uint32_t t = tail + n;
        if (t >= a->size) t -= a->size;
        a->tail = t;
        __DMB();

        reclaimed_bytes += n;
        cnt++;

        if (need_plus_guard) {
            uint32_t head2 = a->head;
            uint32_t tail2 = a->tail;
            uint32_t free_bytes = (head2 >= tail2) ? (a->size - (head2 - tail2)) : (tail2 - head2);
            if (free_bytes >= need_plus_guard) break;
        }
    }

    if (reclaimed_bytes) {
        DEBUG_DUMP(IOT_LOG_DEBUG, "hub_spsc_reclaim(%s): reclaimed=%lu bytes, tail=%lu\r\n",
                   __arena_name(a), (unsigned long)reclaimed_bytes, (unsigned long)a->tail);
    }
}


void hub_spsc_reclaim_all(hub_spsc_arena_t *a)
{
    hub_spsc_reclaim_fast(a, 0, 0);
}

void hub_spsc_reclaim_for_alloc(hub_spsc_arena_t *a, uint32_t need, uint32_t guard, uint32_t budget_chunks)
{
    uint32_t cnt  = 0;
    const uint32_t size = a->size;

    for (;;) {
        if (budget_chunks && cnt >= budget_chunks) break;

        uint32_t head = a->head;
        uint32_t tail = a->tail;
        if (tail == head) break;

        uint32_t free_bytes = (head >= tail) ? (size - (head - tail)) : (tail - head);

        if (head >= tail) {
            uint32_t end_space = size - head;
            if ((end_space >= need) && (free_bytes >= (need + guard))) break;
        } else {
            uint32_t span = tail - head;
            if (span >= (need + guard)) break;
        }

        hub_blk_hdr_t *h = (hub_blk_hdr_t *)(void*)(a->base + tail);
        __DMB();

        if (h->magic != HUB_BLK_MAGIC) {
            if (head < tail) {
                uint32_t gap = size - tail;
                if ((gap & (HUB_DMA_ALIGN - 1U)) == 0U) {
                    a->tail = 0;
                    __DMB();
                    DEBUG_DUMP(IOT_LOG_DEBUG,
                               "hub_spsc_reclaim(%s): wrap-gap @off=%lu → tail=0\r\n",
                               (a==&g_hub_rx_arena)?"RX":"TX", (unsigned long)tail);
                    continue;
                }
            }
            break;
        }

        if (h->done != HUB_DONE_MAGIC) break;

        uint32_t n = h->len_aligned + HUB_HDR_SIZE;
        uint32_t t = tail + n; if (t >= size) t -= size;
        a->tail = t;
        __DMB();
        cnt++;
    }
}

void *hub_spsc_alloc_frame(hub_spsc_arena_t *a, uint32_t payload_len, uint32_t guard)
{
    payload_len = HUB_ALIGN_UP(payload_len);
    guard       = HUB_ALIGN_UP(guard);

    const uint32_t need = HUB_HDR_SIZE + payload_len;

    hub_spsc_reclaim_for_alloc(a, need, guard, 0);

    uint32_t head = a->head;
    uint32_t tail = a->tail;
    const uint32_t size = a->size;

    if (need > size) {
        DEBUG_DUMP(IOT_LOG_ERR, "hub_spsc_alloc(%s): req too big need=%lu > size=%lu\r\n",
                   __arena_name(a), (unsigned long)need, (unsigned long)size);
        return NULL;
    }

    uint32_t free_bytes = (head >= tail) ? (size - (head - tail)) : (tail - head);

    if (head < tail) {
        uint32_t space = tail - head;
        if (need + guard <= space) {
            hub_blk_hdr_t *h = hdr_at(a, head);
            h->len_aligned = payload_len;
            h->seq         = (a == &g_hub_rx_arena) ? ++g_rx_seq : ++g_tx_seq;
            h->done        = 0;
            h->magic       = HUB_BLK_MAGIC;
            __DMB();
            a->head = head + need;
            __DMB();
            return (uint8_t*)h + HUB_HDR_SIZE;
        }
        DEBUG_DUMP(IOT_LOG_ERR, "hub_spsc_alloc(%s): head<tail no space (need=%lu, guard=%lu, head=%lu, tail=%lu, size=%lu)\r\n",
                   __arena_name(a),(unsigned long)need,(unsigned long)guard,(unsigned long)head,(unsigned long)tail,(unsigned long)size);
        return NULL;
    }

    uint32_t end_space = size - head;
    if (need + guard <= end_space) {
        hub_blk_hdr_t *h = hdr_at(a, head);
        h->len_aligned = payload_len;
        h->seq         = (a == &g_hub_rx_arena) ? ++g_rx_seq : ++g_tx_seq;
        h->done        = 0;
        h->magic       = HUB_BLK_MAGIC;
        __DMB();
        uint32_t head2 = head + need;
        if (head2 >= size) head2 -= size;
        a->head = head2;
        __DMB();
        return (uint8_t*)h + HUB_HDR_SIZE;
    }

    if ((end_space >= need) && (free_bytes >= (need + guard))) {
        hub_blk_hdr_t *h = hdr_at(a, head);
        h->len_aligned = payload_len;
        h->seq         = (a == &g_hub_rx_arena) ? ++g_rx_seq : ++g_tx_seq;
        h->done        = 0;
        h->magic       = HUB_BLK_MAGIC;
        __DMB();
        uint32_t head2 = head + need;
        a->head = (head2 >= size) ? 0 : head2;
        __DMB();
        return (uint8_t*)h + HUB_HDR_SIZE;
    }

    if ((need <= tail) && (free_bytes >= (need + guard))) {
        uint32_t slack = end_space;
        if (slack >= HUB_HDR_SIZE) {
            hub_blk_hdr_t *pad = hdr_at(a, head);
            pad->len_aligned = HUB_ALIGN_UP(slack - HUB_HDR_SIZE);
            pad->seq         = 0;
            pad->done        = HUB_DONE_MAGIC;
            pad->magic       = HUB_BLK_MAGIC;
            __DMB();
            DEBUG_DUMP(IOT_LOG_ALL, "hub_spsc_alloc(%s): PAD off=%lu len=%lu → wrap to 0\r\n",
                       __arena_name(a), (unsigned long)head, (unsigned long)pad->len_aligned);
        }
        hub_blk_hdr_t *h0 = hdr_at(a, 0);
        h0->len_aligned = payload_len;
        h0->seq         = (a == &g_hub_rx_arena) ? ++g_rx_seq : ++g_tx_seq;
        h0->done        = 0;
        h0->magic       = HUB_BLK_MAGIC;
        __DMB();
        a->head = need;
        __DMB();
        return (uint8_t*)h0 + HUB_HDR_SIZE;
    }

    hub_blk_hdr_t *ht = hdr_at(a, tail);
    DEBUG_DUMP(IOT_LOG_ERR,
        "hub_spsc_alloc(%s): NO SPACE (need=%lu, guard=%lu, head=%lu, tail=%lu, size=%lu); "
        "tail-hdr: magic=0x%08lX done=0x%08lX len=%lu\r\n",
        __arena_name(a), (unsigned long)need, (unsigned long)guard,
        (unsigned long)head, (unsigned long)tail, (unsigned long)size,
        (unsigned long)ht->magic, (unsigned long)ht->done, (unsigned long)ht->len_aligned);
    return NULL;
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

void hub_spsc_done(hub_spsc_arena_t *a, void *payload_ptr)
{
    if (!payload_ptr) return;
    hub_blk_hdr_t *h = (hub_blk_hdr_t *)((uint8_t*)payload_ptr - HUB_HDR_SIZE);
    if (h->magic != HUB_BLK_MAGIC) {
        DEBUG_DUMP(IOT_LOG_ERR, "hub_spsc_done(%s): BAD MAGIC payload=%p\r\n",
                   __arena_name(a), payload_ptr);
        return;
    }
    __DMB();
    h->done = HUB_DONE_MAGIC;
    __DMB();
    DEBUG_DUMP(IOT_LOG_ALL, "hub_spsc_done(%s): seq=%lu len=%lu payload=%p\r\n",
               __arena_name(a), (unsigned long)h->seq, (unsigned long)h->len_aligned, payload_ptr);
}

void hub_spsc_reset(hub_spsc_arena_t *a)
{
    a->tail = a->head;
    __DMB();
}

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
    if (b->freeb.size >= need + HUB_MEM_MIN_BLK_SZ) {
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
    uint32_t total = HUB_ALIGN_UP(HUB_MEM_HDR_BLK_SZ + need);
    if (total < (HUB_MEM_HDR_BLK_SZ + HUB_MEM_MIN_BLK_SZ)) total = HUB_MEM_HDR_BLK_SZ + HUB_MEM_MIN_BLK_SZ;
    total = HUB_ALIGN_UP(total);

    heap_lock();

    hub_block_t *prev = NULL, *cur = g_heap.free_list;
    while (cur) {
        uintptr_t cur_addr = (uintptr_t)cur;
        uintptr_t payload  = HUB_ALIGN_UP(cur_addr + HUB_MEM_HDR_BLK_SZ);
        uint32_t front_gap = (uint32_t)(payload - (cur_addr + HUB_MEM_HDR_BLK_SZ));
        uint32_t need2 = HUB_ALIGN_UP(HUB_MEM_HDR_BLK_SZ + front_gap + need);
        if (need2 > total) total = need2;

        if (cur->freeb.size >= total) {
            if (prev) prev->freeb.next = split_block(cur, total);
            else      g_heap.free_list = split_block(cur, total);

            cur->alloc.size  = total;
            cur->alloc.magic = HUB_MEM_HEAP_MAGIC;

            void *ret = (void*)((uint8_t*)cur + HUB_MEM_HDR_BLK_SZ);
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

#if 0
void *hub_heap_calloc(size_t cnt, size_t elemsz)
{
    size_t n = cnt * elemsz;
    void *p = hub_heap_alloc(n);
    if (p) memset(p, 0, n);
    return p;
}
#endif


void hub_heap_free(void *p)
{
    if (!p) return;
    hub_block_t *b = (hub_block_t*)((uint8_t*)p - HUB_MEM_HDR_BLK_SZ);
    if (b->alloc.magic != HUB_MEM_HEAP_MAGIC) return;
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

void hub_mem_init(void)
{
    uintptr_t rx_base = HUB_SDRAM_BASE;
    uintptr_t tx_base = rx_base + HUB_I2C_RX_BYTES_DEFAULT;
    uintptr_t heap_base = tx_base + HUB_I2C_TX_BYTES_DEFAULT;

    hub_spsc_init(&g_hub_rx_arena, rx_base,  HUB_I2C_RX_BYTES_DEFAULT);
    hub_spsc_init(&g_hub_tx_arena, tx_base,  HUB_I2C_TX_BYTES_DEFAULT);

    uint32_t heap_bytes = (uint32_t)(HUB_SDRAM_SIZE - (heap_base - HUB_SDRAM_BASE));
    heap_init_region((void*)heap_base, heap_bytes);

    static UINT mtx_inited = 0;
    if (!mtx_inited) {
        tx_mutex_create(&g_heap.mtx, "hub_heap_mtx", TX_INHERIT);
        mtx_inited = 1;
    }
}
