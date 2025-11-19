#ifndef MEMORY_H
#define MEMORY_H

#include "c_types.h"

#define MEM_BLOCK_SIZE		256
#define MEM_TOTAL		81920
#define MEM_TOTAL_BLOCKS	(MEM_TOTAL / MEM_BLOCK_SIZE)

typedef struct
{

} block_allocator;

#endif
