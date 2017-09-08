#include "heap_memory.h"

typedef struct mem_tag {
  memory_t bit_and_size;
} mem_tag_t;

//////////////////////////////////////////////////////////////////////////

#define HEAP_SIZE 2048
#define mem_block_allocate_bit 0x8000000000000000U
#define is_block_allocated(x) ((x & mem_block_allocate_bit))

#define mem_tag_size(x) ((x & ~mem_block_allocate_bit))
#define mem_block_allocate(mem_block, size) (mem_block)=(size)|mem_block_allocate_bit
#define mem_tag_free(mem_block) (mem_block &= ~mem_block_allocate_bit )

static uint8_t g_heap[HEAP_SIZE] __attribute__((section(".heap_memory"))) = {0};
static uint8_t* g_heap_end = g_heap + HEAP_SIZE;

void
hm_init() {
  mem_tag_t* mt = (mem_tag_t*)g_heap;  
  mt->bit_and_size = g_heap_end - sizeof(mem_tag_t) - g_heap;
}
//////////////////////////////////////////////////////////////////////////

memory_t hm_malloc(memory_t size) {
  mem_tag_t* rv = (mem_tag_t*)g_heap;
  size = (size + 1) & (~0x01); //if (size % 2) ++size; we will operate with even addresses.
  for(;;) {
    if (!is_block_allocated(rv->bit_and_size) && (rv->bit_and_size > size)) {
      if (rv->bit_and_size > (size + sizeof(mem_tag_t))) {
        ((mem_tag_t*)((memory_t)rv + size + sizeof(mem_tag_t)))->bit_and_size =
            rv->bit_and_size - (size + sizeof(mem_tag_t));
      } else {
        size = rv->bit_and_size; //if we at the end of memory
      }
      mem_block_allocate(rv->bit_and_size, size); //mark block allocated.
      return (memory_t)rv + sizeof(mem_tag_t);
    }

    *(memory_t*)&rv = (memory_t)rv + mem_tag_size(rv->bit_and_size) + sizeof(mem_tag_t); //change rv pointer.
    if (((memory_t)rv & ~mem_block_allocate_bit) >= (memory_t)g_heap_end)
      return 0; //we haven't enough memory
  }
}
//////////////////////////////////////////////////////////////////////////

//if addr is not valid then behavior will be unpredictable
void
hm_free(memory_t p) {
  mem_tag_t* next = (mem_tag_t*) (p - sizeof(mem_tag_t));
  mem_tag_free(next->bit_and_size);
  p = (memory_t)g_heap;
  for (;;) {
    next = (mem_tag_t*) (p + mem_tag_size(((mem_tag_t*)p)->bit_and_size) + sizeof(mem_tag_t));

    if (!is_block_allocated(((mem_tag_t*)p)->bit_and_size)) {
      if ((uint64_t)next >= (memory_t)g_heap_end) break;

      if (!is_block_allocated(next->bit_and_size)) {
        ((mem_tag_t*)p)->bit_and_size += next->bit_and_size + sizeof(mem_tag_t);
      } else {
        p = (memory_t)next;
      }
    } else {
      p = (memory_t)next;
    }

    if (p >= (memory_t)g_heap_end) return;
  }
}
//////////////////////////////////////////////////////////////////////////
