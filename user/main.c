#include "esp8266.h"

#define PIN_LED			2

int
main(void)
{
	PIN_FUNC_SELECT(PERI_IO_MUX_GPIO2_U, 0);
	PIN_FUNC_SELECT(PERI_IO_MUX_GPIO5_U, 0);
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
