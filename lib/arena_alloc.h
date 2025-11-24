#ifndef ARENA_ALLOC_H
#define ARENA_ALLOC_H

// TODO: When multitasking will be implemented, maybe make this task-safe

#ifndef USE_MEM_MALLOC

#include "memory.h"
#define MEM_MALLOC()	memory_allocate_block()
#define MEM_FREE(a)	memory_free_block((void*)a)

#else

#include "esp8266.h"
#define MEM_MALLOC()	mem_malloc(0x1000 / 2)
#define MEM_FREE(a)	mem_free((void*)a)

#endif

#define ALIGN_DOWN_4(a)	((a) & ~3)
#define ALIGN_UP_4(a)	(((a) + 3) & ~3)

/* MINIMAL ARENA ALLOCATOR IMPLEMENTATION
 * 
 * This implementation aims for speed with fast copies and minimal size, implementing only necessary functions.
 * It is header only so be carefull if you intend to include this file many times.
 * I have second thoughts on if I should compile it in a c file or not.
 *
 * USAGE
 *
 * Init:
 * Initalize an arena by calling arena_new(size). Allocate the arena on the stack.
 *
 * Create:
 * Create an arena from an array of bytes by calling arena_from_ptr.
 * 
 * Push and pop:
 * Push reserves size space and aligns up that size so that every stucture in the arena are aligned.
 * To use pop, you need to save the state of the stack pointer (.sp) 
 * before pushing to the arena and pass it as the mark argument to the macro.
 *
 * Full example:
 *
 * void
 * some_func_that_needs_heap_allocation(Arena* arena)
 * {
 * 		u32 mark = arena->sp;
 * 		some_struct* my_struct_ptr = (some_struct*)arena_push(arena, sizeof(some_struct));
 *
 * 		// ... do some stuff with my_struct_ptr ...
 *
 * 		arena_pop(arena, mark);
 * }
 *
 * void
 * main(void)
 * {
 * 		Arena	my_arena = arena_new(0x1000); // 4kB
 *
 * 		some_func_that_needs_heap_allocation(&my_arena);
 *
 *		return 0;
 * }
 *
 * */

typedef struct
{
	u8*	start;
	u8*	end;
	u32	sp;
} Arena;

INTERNAL inline Arena*
arena_from_ptr(void* ptr)
{
	if (!ptr) return 0;

	Arena* new_arena = ptr;

	new_arena->start = (u8*)((uptr)new_arena + sizeof(Arena));
	new_arena->end	= (u8*)((uptr)ptr + MEM_BLOCK_SIZE);
	new_arena->sp 	= 0;

	return new_arena;
}

INTERNAL inline Arena* ALWAYS_INLINE_ATTR
arena_new(void)
{
	return arena_from_ptr(MEM_MALLOC());
}

INTERNAL inline u8*
arena_push(Arena* arena, u32 object_size)
{
	if (!arena || !object_size) return 0;
	object_size = ALIGN_UP_4(object_size);
	if (object_size >= (uptr)arena->end - arena->sp) return 0;
	
	u8*	res = arena->start + arena->sp;

	arena->sp += object_size;
	return res;
}

INTERNAL inline void ALWAYS_INLINE_ATTR
arena_pop(Arena* arena, u32 mark)
{
	if (!arena || mark > arena->sp) return;
	arena->sp = mark;
}

INTERNAL inline s32 ALWAYS_INLINE_ATTR
arena_delete(Arena* arena)
{
	return MEM_FREE(arena);
}


#endif
