#ifndef HEAP_MEMORY_H
#define HEAP_MEMORY_H

#include <stdint.h>

typedef uint64_t memory_t;

void hm_init();
memory_t hm_malloc(memory_t size);
void hm_free(memory_t p);

#endif  // HEAP_MEMORY_H
