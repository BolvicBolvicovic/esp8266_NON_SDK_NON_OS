#ifndef ARENA_ALLOC_H
#define ARENA_ALLOC_H

// TODO: When multitasking will be implemented, maybe make this task-safe

#include "c_types.h"

#ifdef UNIT_TESTS
extern u8*	_heap_start;
#define ARENA_HEAP_BASE	((u8*)(ALIGN_UP_4((uptr)_heap_start)))
#else
extern u8	_heap_start;
#define ARENA_HEAP_BASE	((u8*)(ALIGN_UP_4((uptr)&_heap_start)))
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
 * Initalize an arena by calling arena_new(size).
 *
 * Alloc:
 * If you want to create a second arena, you need to pass the first arena you allocated as a parameter to arena_create.
 * This avoids handling the heap as much as possible.
 * That means that future allocator implementations might need to use the same idea.
 * However, this is not a fixed idea and is subject for refactoring with the creation of a minimal heap managment API.
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
arena_from_ptr(void* ptr, u32 size)
{
	if (size <= sizeof(Arena)) return 0;

	Arena* new_arena = ptr ? (Arena*)ptr : (Arena*)ARENA_HEAP_BASE;

	new_arena->start = (u8*)((uptr)new_arena + sizeof(Arena));
	new_arena->end	= new_arena->start + ALIGN_UP_4(size + sizeof(Arena));
	new_arena->sp 	= 0;

	return new_arena;
}

INTERNAL inline Arena* ALWAYS_INLINE_ATTR
arena_new(u32 size)
{
	return arena_from_ptr(0, size);
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

#endif
