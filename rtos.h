#ifndef RTOS_H
#define RTOS_H

#include <stdint.h>

typedef uint32_t	u32;
typedef int32_t		s32;

extern void	ets_printf(const char* fmt, ...);
extern void	ets_delay_us(u32 us);
extern void	ets_install_putc1(void (*)(char));
extern void	uart_tx_one_char(char);

#define INTERNAL		static
#define BIT(a)			(1 << (a))
#define REG32(a)		(*(volatile u32*)(a))

#define GPIO_OUT_REG		REG32(0x60000300)
#define GPIO_OUT_SET_REG	REG32(0x60000304)
#define GPIO_OUT_CLR_REG	REG32(0x60000308)
#define GPIO_ENABLE_REG		REG32(0x6000030C)

#define UART_BASE_REG(a)	(0x60000000 + (a) * 0xf00)
#define UART_CLKDIV(a)		(UART_BASE_REG(a) + 0x14)

#endif
