#if 0
#include <errno.h>
#include <stdint.h>
#include <stddef.h>
extern uint8_t _tx_heap_start;
extern uint8_t _tx_heap_end;

static uint8_t *heap_curr = &_tx_heap_start;

void *_sbrk(ptrdiff_t incr)
{
    uint8_t *prev = heap_curr;
    uint8_t *next = heap_curr + incr;

    if (next > &_tx_heap_end) {
        errno = ENOMEM;
        return (void *)-1;
    }

    heap_curr = next;
    return (void *)prev;
}

#if defined(__PICOLIBC__)
  __strong_reference(_sbrk, sbrk);
#endif
#endif
