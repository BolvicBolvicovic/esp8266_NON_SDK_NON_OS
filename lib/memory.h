#ifndef MEMORY_H
#define MEMORY_H

#include "c_types.h"

#define MEM_BLOCK_SIZE		0x1000
#define MEM_TOTAL		81920
#define MEM_TOTAL_BLOCKS	(MEM_TOTAL / MEM_BLOCK_SIZE)

typedef u32 memory_free_block_list;

void	memory_init(void);
void*	memory_allocate_block(void);
s32	memory_free_block(void* block);

#endif
