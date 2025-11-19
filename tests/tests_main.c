#include "tests_lib.h"

int
main(void)
{
	_heap_start = (u8*)malloc(TESTER_HEAP_SIZE);

	assert(_heap_start);

	tests_arena_alloc();

	return 0;
}
