#ifndef C_TYPES_H
#define C_TYPES_H

#include <stdint.h>

#define ALWAYS_INLINE_ATTR	__attribute__((__always_inline__))
#define ALIGNED_4		__attribute__((aligned(4)))
#define INTERNAL	static
#define GLOBAL		static

typedef uint8_t		u8;
typedef uint32_t	u32;
typedef uint32_t	uptr;
typedef int32_t		s32;

#endif
