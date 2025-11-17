#include "../rtos.h"

#define PIN_LED		2
#define BAUDRATE	115200

INTERNAL void
my_putc(char c)
{
	uart_tx_one_char(c);
}

int
main(void)
{
	ets_update_cpu_frequency(80);
	REG32(UART_CLKDIV(0)) = 80000000 / BAUDRATE;
	ets_install_putc1(my_putc);
	GPIO_ENABLE_REG |= BIT(PIN_LED);
	
	ets_printf("\n\nLet's get started!\n");

	for (;;)
	{
		GPIO_OUT_CLR_REG = BIT(PIN_LED);
		ets_delay_us(500000);
		ets_printf("ON\n");

		GPIO_OUT_SET_REG = BIT(PIN_LED);
		ets_delay_us(500000);
		ets_printf("OFF\n");
	}

	return 0;
}
