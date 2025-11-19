#ifndef ESP8266_H
#define ESP8266_H

#include "c_types.h"

extern void		ets_install_putc1(void (*putc)(u8 c));
extern void		ets_printf(const char* fmt, ...);
extern void		ets_delay_us(u32 us);
extern void		ets_update_cpu_frequency(u32 mhz);
extern void		uart_div_modify(u8 uart, u32 div);
extern void		rom_i2c_writeReg(u8 slave_addr, u8 reg_addr_len, u8 data_len, u32 data);

#define BIT(a)			(1 << (a))
#define REG32(a)		(*(volatile u32*)(a))
#define CPU_FREQ_MHZ	80
#define BAUDRATE		115200
#define IRAM_ATTR		__attribute__((section(".iram1")))


#define CLEAR_PERI_REG_MASK(reg, mask)	(REG32(reg) = REG32(reg) & (~(mask)))
#define SET_PERI_REG_MASK(reg, mask)	(REG32(reg) = REG32(reg) | (mask))

#define PERI_IO_MUX			0x60000800
#define PERI_IO_MUX_FUNC	0x13
#define PERI_IO_MUX_FUNC_S	4
#define PERI_IO_MUX_PULLUP	BIT(7)
#define PERI_IO_MUX_U0TXD_U	(PERI_IO_MUX + 0x18)
#define PERI_IO_MUX_GPIO2_U	(PERI_IO_MUX + 0x38)
#define PERI_IO_MUX_GPIO5_U	(PERI_IO_MUX + 0x40)

#define PIN_PULLUP_DISABLE(pin_name)	CLEAR_PERI_REG_MASK(pin_name, PERI_IO_MUX_PULLUP)
#define PIN_FUNC_SELECT(pin_name, func) \
    do { \
        u32 v = REG32(pin_name); \
        v &= ~(PERI_IO_MUX_FUNC << PERI_IO_MUX_FUNC_S); \
        v |= ((func & PERI_IO_MUX_FUNC) << PERI_IO_MUX_FUNC_S); \
        REG32(pin_name) = v; \
    } while (0)

#define GPIO_OUT_REG			REG32(0x60000300)
#define GPIO_OUT_SET_REG		REG32(0x60000304)
#define GPIO_OUT_CLR_REG		REG32(0x60000308)
#define GPIO_ENABLE_REG			REG32(0x6000030C)

#define UART_BASE_REG(u)        (0x60000000 + (u) * 0xF00)
#define UART_FIFO(u)            (UART_BASE_REG(u) + 0x00)
#define UART_STATUS(u)          (UART_BASE_REG(u) + 0x1C)
#define UART_TXFIFO_CNT         0x000000FF
#define UART_TXFIFO_CNT_S       16
#define UART_STOP_BIT_NUM		3
#define UART_STOP_BIT_NUM_S		4
#define UART_PARITY_ENABLE		BIT(1)
#define UART_PARITY			BIT(0)

#endif
