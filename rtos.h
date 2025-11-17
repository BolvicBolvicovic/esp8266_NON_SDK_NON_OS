#ifndef RTOS_H
#define RTOS_H

#include <stdint.h>

typedef uint32_t u32;

extern void	ets_printf(const char* fmt, ...);
extern void	ets_delay_us(u32 us);

#define GPIO_OUT_REG		(*(volatile u32*)0x60000300)
#define GPIO_OUT_SET_REG	(*(volatile u32*)0x60000304)
#define GPIO_OUT_CLR_REG	(*(volatile u32*)0x60000308)
#define GPIO_ENABLE_REG		(*(volatile u32*)0x6000030C)
#define BIT(a)			(1 << (a))

#endif
