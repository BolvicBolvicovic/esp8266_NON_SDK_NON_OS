#include "tests_lib.h"
	
void
tests_memory_basic_cycle(void)
{
	u8*	block = (u8*)memory_allocate_block();
	assert(block);

	for (int i = 0; i < MEM_BLOCK_SIZE; i++)
		block[i] = i % 255;

	for (int i = 0; i < MEM_BLOCK_SIZE; i++)
		assert(block[i] == i % 255);


	assert(!memory_free_block(block + 1));
	assert(memory_free_block(block));
}

void
tests_memory_max_blocks(void)
{
	u8*	all_blocks[MEM_TOTAL_BLOCKS];

	for (int i = 0; i < MEM_TOTAL_BLOCKS; i++)
	{
		all_blocks[i] = (u8*)memory_allocate_block();
		assert(all_blocks[i]);
	}

	assert(!memory_allocate_block());

	for (int i = 0; i < MEM_TOTAL_BLOCKS; i++)
		assert(memory_free_block((void*)all_blocks[i]));
	
	for (int i = 0; i < MEM_TOTAL_BLOCKS; i++)
	{
		all_blocks[i] = (u8*)memory_allocate_block();
		assert(all_blocks[i]);
	}

	for (int i = 0; i < MEM_TOTAL_BLOCKS; i++)
		assert(memory_free_block((void*)all_blocks[i]));
}

void
tests_memory(void)
{
	memory_init();
	tests_memory_basic_cycle();
	tests_memory_max_blocks();
}
