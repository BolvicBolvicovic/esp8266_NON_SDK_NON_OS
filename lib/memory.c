#include "memory.h"

static u8			_memory_pool[MEM_TOTAL] ALIGNED_4 = {0};
static memory_free_block_list*	_memory_free_block_list_head = (memory_free_block_list*)_memory_pool;
static memory_free_block_list*	_memory_free_block_list_tail = (memory_free_block_list*)(_memory_pool + MEM_TOTAL - MEM_BLOCK_SIZE);

void
memory_init(void)
{
	for (u32 i = 0; i < MEM_TOTAL_BLOCKS - 1; i++)
	{
		_memory_free_block_list_head[i * (MEM_BLOCK_SIZE / sizeof(memory_free_block_list))] =
			(memory_free_block_list)&_memory_free_block_list_head
				[(i + 1) * (MEM_BLOCK_SIZE / sizeof(memory_free_block_list))];
	}
}

void*
memory_allocate_block(void)
{
	if (!_memory_free_block_list_head) return 0;
	if (_memory_free_block_list_head == _memory_free_block_list_tail)
		_memory_free_block_list_tail = 0;

	void*	block = (void*)_memory_free_block_list_head;
	_memory_free_block_list_head = (memory_free_block_list*)*_memory_free_block_list_head;
	return block;
}

s32
memory_free_block(void* block)
{
	if (!block 
		|| ((uptr)block - (uptr)_memory_pool) % MEM_BLOCK_SIZE
		|| (uptr)block < (uptr)_memory_pool 
		|| (uptr)block > (uptr)_memory_pool + MEM_TOTAL)
		return 0;
	if (_memory_free_block_list_tail)
		*_memory_free_block_list_tail = (memory_free_block_list)block;
	if (!_memory_free_block_list_head)
		_memory_free_block_list_head = (memory_free_block_list*)block;

	_memory_free_block_list_tail = (memory_free_block_list*)block;
	*_memory_free_block_list_tail = 0;
	return 1;
}
