#ifndef ARENA_ALLOC_H
#define ARENA_ALLOC_H

#include "c_types.h"

extern u8	_heap_start;

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
 * Initalize the first arena by calling ARENA_INIT(size).
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
 * */

typedef struct
{
	u8*	start;
	u8*	end;
	u32	sp;
} Arena;

INTERNAL inline Arena
arena_create(Arena* prev, u32 size)
{
	Arena new_arena;

	new_arena.start = prev ? prev->end : ALIGN_UP_4((uptr)&_heap_start);
	new_arena.end	= new_arena.start + ALIGN_UP_4(size);
	new_arena.sp 	= 0;

	return new_arena;
}

#define ARENA_INIT(size)	(arena_create(0, size))

INTERNAL inline u8*
arena_push(Arena* arena, void* object, u32 object_size)
{
	if (!arena || !object) return 0;
	object_size	= ALIGN_UP_4(object_size);
	if (object_size >= (uptr)arena->end - arena->sp) return 0;
	
	u8*	res = arena->start + arena->sp;

	arena->sp	+= object_size;
	return res;
}

// Note: arena MUST be of type Arena*
#define ARENA_POP(arena, mark)	(arena->sp = mark)

#endif
