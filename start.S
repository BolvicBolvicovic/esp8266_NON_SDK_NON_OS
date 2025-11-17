.extern		main

.section	.UserEnter.text, "ax"
.global		call_user_start1
.type		call_user_start1, @function
.align		4
.literal_position

call_user_start1:
	movi	a2, _bss_start
	movi	a3, _bss_end
	movi	a0, 0
	bgeu	a2, a3, 1f
0:	s32i	a0, a2, 0
	addi	a2, a2, 4
	bltu	a2, a3, 0b
1:	call0	main
.hang:	waiti	0
	j	.hang
