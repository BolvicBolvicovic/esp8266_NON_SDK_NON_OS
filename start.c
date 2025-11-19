#include "esp8266.h"

extern void	main(void);
extern u32	_bss_start;
extern u32	_bss_end;

void
call_user_start1()
{
	rom_i2c_writeReg(103, 4, 1, 0x88);
	rom_i2c_writeReg(103, 4, 2, 0x91);
 	ets_update_cpu_frequency(CPU_FREQ_MHZ);

	for (u32* addr = &_bss_start; addr < &_bss_end; addr++) *addr = 0;

	PIN_PULLUP_DISABLE(PERI_IO_MUX_U0TXD_U);
	PIN_FUNC_SELECT(PERI_IO_MUX_U0TXD_U, 0);
	
	uart_div_modify(0, (CPU_FREQ_MHZ * 1000000) / BAUDRATE);

	main();
}
