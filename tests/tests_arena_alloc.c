#include "tests_lib.h"

INTERNAL void
tests_arena_alloc_basic_cycle()
{
	Arena*	arena = arena_init(0x1000);
	u8*	start = arena->start;
	u8*	end = arena->end;
	u32	mark = arena->sp;

	assert(_heap_start);
	assert(start);
	assert(end);
	assert((uptr)start != (uptr)arena);

	s32*	s32_array = (s32*)arena_push(arena, sizeof(int) * 15);

	assert(s32_array);

	for (s32 i = 0; i < 15; i++)
	{
		s32_array[i] = i;
	}

	for (s32 i = 0; i < 15; i++)
	{
		assert(s32_array[i] == i);
	}

	arena_pop(arena, mark);
	
	assert(arena->sp == 0);
	assert(arena->start == start);
	assert(arena->end == end);
}

INTERNAL void
tests_arena_alloc_null_size_init_create()
{
	Arena*	arena = arena_init(0);
	assert(arena == 0);
}

INTERNAL void
tests_arena_alloc_null_push()
{
	Arena*	arena = arena_init(0x1000);
	s32*	s32_array = (s32*)arena_push(arena, 0);
	assert(s32_array == 0);
}

INTERNAL void
tests_arena_alloc_pop_bigger_mark()
{
	Arena*	arena = arena_init(0x1000);
	arena_push(arena, sizeof(s32) * 10);
	u32	mark = arena->sp;
	
	arena_pop(arena, mark + 1);
	assert(arena->sp == mark); // Unchanged
}

void
tests_arena_alloc(void)
{
	tests_arena_alloc_basic_cycle();
	tests_arena_alloc_null_size_init_create();
	tests_arena_alloc_null_push();
	tests_arena_alloc_pop_bigger_mark();
}
