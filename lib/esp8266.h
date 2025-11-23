#ifndef ESP8266_H
#define ESP8266_H

#include "c_types.h"

#define BIT(a)			(1 << (a))
#define REG32(a)		(*(volatile u32*)(a))
#define CPU_FREQ_MHZ		80
#define BAUDRATE		115200
#define IRAM_ATTR		__attribute__((section(".iram1")))


#define CLEAR_PERI_REG_MASK(reg, mask)	(REG32(reg) = REG32(reg) & (~(mask)))
#define SET_PERI_REG_MASK(reg, mask)	(REG32(reg) = REG32(reg) | (mask))

#define PERI_IO_MUX		0x60000800
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

#define GPIO_OUT_REG		REG32(0x60000300)
#define GPIO_OUT_SET_REG	REG32(0x60000304)
#define GPIO_OUT_CLR_REG	REG32(0x60000308)
#define GPIO_ENABLE_REG		REG32(0x6000030C)

#define UART_BASE_REG(u)        (0x60000000 + (u) * 0xF00)
#define UART_FIFO(u)            (UART_BASE_REG(u) + 0x00)
#define UART_STATUS(u)          (UART_BASE_REG(u) + 0x1C)
#define UART_TXFIFO_CNT         0x000000FF
#define UART_TXFIFO_CNT_S       16
#define UART_STOP_BIT_NUM	3
#define UART_STOP_BIT_NUM_S	4
#define UART_PARITY_ENABLE	BIT(1)
#define UART_PARITY		BIT(0)

/* --- BOOTLOADER FUNCTIONS --- */

/* ETS TYPES */

/* Name: ets_event_t
 * Description: event structure used by the ets_event related functions.
 * Based on the ESP8266 NON SDK from Espressif.
 * */
typedef struct
{
	u32	signal;
	u32	parameter;
} ets_event_t;


/* Name: ets_timer_t
 * Description: timer structure used by the ets_timer functions and by the watchdog internally.
 * Based on the ESP8266 NON SDK from Espressif.
 * UNSURE 
 * */
typedef struct ets_timer_s
{
	struct ets_timer_s	*next;
	u32			expire;
	u32			period;
	void	(*ets_timer_func)(void* arg);
	void*			arg;
/* COULD BE --- I am questionning this after looking at the assembly of ets_timer_arm */
/*	void	(*ets_timer_func)(void*);
 *	void*	arg;
 *	u32	period;
 *	u8	flags;
 *	u8	rsvd_a14;
 *	u8	rsvd_a15;
 * */
} ets_timer_t;


/* ETS FUNCTIONS */

/* Name: ets_bzero
 * Address: 0x40002ae8
 * Description: zeros n bytes of array s.
 * */
extern void	ets_bzero(void* s, u32 n);

/* Name: ets_char2xdigit
 * Address: 0x40002b74
 * Description: converts a char to an hex digit signed integer.
 * */
extern s32	ets_char2xdigit(char c);

/* Name: ets_delay_us
 * Address: 0x40002ecc
 * Description: delays for input micro seconds.
 * */
extern void	ets_delay_us(u32 us);

/* Name: ets_enter_sleep
 * Address: 0x400027b8
 * Description: wrapper around ets_set_idle_cb using rtc_enter_sleep as a parameter and enter sleep mode.
 * */
extern void	ets_enter_sleep(void);

/* Name: ets_external_printf
 * Address: 0x40002578
 * Description: same implementation as ets_printf.
 * I suppose that it is there until one replaces it with ets_install_external_printf
 * so one can have one's own printf in the ROM?
 * */
extern s32	ets_external_printf(const char* fmt, ...);

/* Name: ets_get_cpu_frequency
 * Address: 0x40002f0c
 * Description: reads from the register at 0x3fffc704 the CPU frequency.
 * */
extern u32	ets_get_cpu_frequency(void);

/* Name: ets_getc
 * Address: 0x40002bcc
 * Description: wrapper around uart_rx_one_char_block.
 * */
extern char	ets_getc(void);

/* Name: ets_install_external_printf
 * Address: 0x40002450
 * Description: wrapper around ets_install_putc2.
 * I am not 100% sure about the functions signature.
 * */
extern void	ets_install_external_printf(void (*external_printf)(const char*));

/* Name: ets_install_putc1
 * Address: 0x4000242c
 * Description: installs a putc function by updating the register at 0x3fffdd48
 * with the input function pointer.
 * */
extern void	ets_install_putc1(void (*putc)(u8 c));

/* Name: ets_install_putc2
 * Address: 0x4000248c
 * Description: installs a putc function by updating the register at 0x3fffdd4c
 * with the input function pointer.
 * */
extern void	ets_install_putc2(void (*putc)(u8 c));

/* Name: ets_install_uart_printf
 * Address: 0x40002438
 * Description: wrapper around ets_install_putc1.
 * I am not 100% sure about the functions signature as it possibly .
 * It seems to load the word _unknown_3 (0xcc1d0040) which I have no idea what that is.
 * */
extern void	ets_install_uart_printf(void (*uart_printf)(const char*));

/* Name: ets_intr_lock
 * Address: 0x40000f74
 * Description: sets the interrupt level to 3. All interrupts under or of level 3 are locked.
 * Sets the register at 0x3fffdcc0 to 3. Maybe remembering the new interrupt level?
 * */
extern void	ets_intr_lock(void);

/* Name: ets_intr_unlock
 * Address: 0x40000f80
 * Description: sets the interrupt level to 0. Effectively enables all interrupts.
 * */
extern void	ets_intr_unlock(void);

/* Name: ets_isr_attach
 * Address: 0x40000f88
 * Description: wrapper around _xtos_set_interrupt_handler_arg, setting up the interrupt handler.
 * */
extern void	ets_isr_attach(s32 interrupt_nb, void* handler, void* args); 

/* Name: ets_isr_mask
 * Address: 0x40000f98
 * Description: wrapper around _xtos_ints_off, clearing the bits in the mask from the interrupt flag.
 * */
extern void	ets_isr_mask(u32 mask);

/* Name: ets_isr_unmask
 * Address: 0x40000fa8
 * Description: wrapper around _xtos_ints_on, setting the bits in the mask from the interrupt flag.
 * */
extern void	ets_isr_unmask(u32 mask);

/* Name: ets_memcmp
 * Address: 0x400018d4
 * Description: wrapper around memcmp.
 * Compares n bytes of b1 and b2.
 * Returns 0 if b1 and b2 are equal, 1 if the different byte of b1 > than byte of b2 else -1.
 * */
extern s32	ets_memcmp(const void* b1, const void* b2, u32 n);

/* Name: ets_memcpy
 * Address: 0x400018b4
 * Description: wrapper around memcpy
 * Copies n bytes from src to dst.
 * Returns dst.
 * */
extern void*	ets_memcpy(void* dst, const void* src, u32 n);

/* Name: ets_memmove
 * Address: 0x400018c4
 * Description: wrapper around memmove.
 * Copies n bytes from src to dst. Handles overlapping cases of dst and src.
 * Returns dst.
 * */
extern void*	ets_memmove(void* dst, const void* src, u32);

/* Name: ets_memset
 * Address: 0x400018a4
 * Description: wrapper around memset. 
 * Sets n bytes of s with c.
 * Returns s.
 * */
extern void*	ets_memset(void* s, s32 c, u32 n);

/* Name: ets_post
 * Address: 0x40000e24
 * Description: seems to post an event of some kind.
 * event_data type is unspecified but the function is accessing one of its field with a left shift of 4.
 * Loads the register 0x3fffdac0 for its operations.
 * Locks interrupts while executing.
 * Seems to return 1 on failure and 0 on success.
 * */
extern s32	ets_post(s32 event_type, void* event_data, s32 priority);

/* Name: ets_printf
 * Address: 0x400024cc
 * Description: prints a formated string. Seems to take a maximum of 5 extra arguments.
 * Does some set up before calling ets_vprintf.
 * Returns the amount of chars printed.
 * */
extern s32	ets_printf(const char* fmt, ...);

/* Name: ets_putc
 * Address: 0x40002be8
 * Description: wrapper around uart_tx_one_char.
 * */
extern void	ets_putc(u8 byte);

/* Name: ets_rtc_int_register
 * Address: 0x40002a40
 * Description: sets the RTC interrupt handler (int 3).
 * I do not know the signature of the handler. Maybe it takes in the stack trace or something?
 * */
extern void	ets_rtc_int_register(void* handler);

/* Name: ets_run
 * Address: 0x40000e04
 * Description: Seems to be a basic setup for a fresh run.
 * Sets byte at register 0x3fffc6fc to 0.
 * Calls .Lfunc003 which seems to setup the environement for the user.
 * Then waits for an interrupt of level 0 before looping.
 * */
extern void	ets_run(void);

/* Name: ets_set_idle_cb
 * Address: 0x40000dc0
 * Description: sets the idle callback at register 0xfffdab0 and its argument at 0xfffdab4.
 * */
extern void	ets_set_idle_cb(void (*idle_cb)(void*), void* args);

/* Name: ets_set_user_start
 * Address: 0x40000fbc
 * Description: sets the user_start function at register 0x3fffdcd0.
 * */
extern void	ets_set_user_start(void (*user_start)(void));

/* Name: ets_str2macaddr
 * Address: 0x40002af8
 * Description: converts a string to a macaddr.
 * Input string must be a valid macaddr. Format supported:
 * 	- "AA:BB:CC:DD:EE:FF"
 * 	- "aa-bb-cc-dd-ee-ff"
 * 	- "AABBCCDDEEFF"
 * Output buffer must be 6 bytes long.
 * On error, returns 0. On success, returns dst.
 * Uses ets_char2xdigit internally to convert chars into value and merge them into bytes 2 by 2.
 * */
extern u8*	ets_str2macaddr(u8* dst, const char* src);

/* Name: ets_strcmp
 * Address: 0x40002aa8
 * Description: wrapper around strcmp.
 * Returns 0 if s1 and s2 are equal, 1 if the different char of s1 > than char of s2 else -1.
 * */
extern s32	ets_strcmp(const char* s1, const char* s2);

/* Name: ets_strcpy
 * Address: 0x40002a88
 * Description: wrapper around strcpy.
 * Returns dst.
 * */
extern char*	ets_strcpy(char* dst, const char* src);

/* Name: ets_strlen
 * Address: 0x40002ac8
 * Description: wrapper around strlen.
 * */
extern u32	ets_strlen(const char* s);

/* Name: ets_strncmp
 * Address: 0x40002ab8
 * Description: wrapper around strncmp.
 * Returns 0 if s1 and s2 are equal, 1 if the different char of s1 > than char of s2 else -1.
 * */
extern s32	ets_strncmp(const char* s1, const char* s2, u32 n);	

/* Name: ets_strncpy
 * Address: 0x40002a98
 * Description: wrapper around strncpy.
 * Returns dst.
 * */
extern char*	ets_strncpy(char* dst, const char* src, u32 n);

/* Name: ets_strstr
 * Address: 0x40002ad8
 * Description: wrapper around strstr.
 * On success: returns the address of the first char of the needle in the haystack.
 * On error: returns 0.
 * */
extern char*	ets_strstr(const char* haystack, const char* needle);

/* Name: ets_task
 * Address: 0x40000dd0
 * Description: registers a cooperative task (to register at 0x3fffdac0?).
 * Task priority must be between 1 and 9.
 * See ets_event_t in ETS TYPES section.
 * Returns the task function pointer (arg0).
 * */
extern void*	ets_task(void (*task)(ets_event_t*), u8 prio, ets_event_t* queue, u8 qlen);

/* Name: ets_timer_arm
 * Address: 0x40002cc4
 * Description: arms (starts) a new ets_timer_t.
 * Calls timer_insert. Should print something at some point.
 * Untested structure.
 * Returns the timer (arg0).
 * */
extern ets_timer_t*	ets_timer_arm(ets_timer_t* timer, u32 timeout_ms, bool repeat);

/* Name: ets_timer_disarm
 * Address: 0x40002d40
 * Description: disarms (stop) the running ets_timer_t.
 * Checks register at 0x3fffddb0 which could be the head of the linked list of timers.
 * Then walks the list until we find the timer we want to stop.
 * If it cannot find the timer to disarm, marks it as invalid and clears expiration.
 * Returns the timer (arg0).
 * */
extern ets_timer_t*	ets_timer_disarm(ets_timer_t* timer);

/* Name: ets_timer_done
 * Address: 0x40002d80
 * Description: disable the timer.
 * Should print something.
 * After halt, calls ets_timer_setfn with (0, 0) to disable the timer.
 * Returns the timer (arg0).
 * */
extern ets_timer_t*	ets_timer_done(ets_timer_t* timer);

/* Name: ets_timer_handler_isr
 * Address: 0x40002da8
 * Description: calls an isr for timers.
 * Seems to take the head of the ets_timer_t at 0x3fffddb0 and call its callback.
 * Also calls timer_insert with ((ets_timer_t*)head)->expire.
 * */
extern void	ets_timer_handler_isr(void*);

/* Name: ets_timer_init
 * Address: 0x40002e68
 * Description: Sets the head of the ets_timer_t list at 0x3fffddb0.
 * Maybe sets 0x40002e24 as its callback by calling ets_isr_attach with (10, 0x40002e24, head)
 * Unmasks the isr with mask 0x400.
 * Calls ets_task with (0x40002e3c, 0x1f, 0x3fffddc0, 4).
 * Sets register at 0x60000630 to 0.
 * Sets register at 0x60000628 to 0x88.
 * Sets register at 0x60000620 to 0.
 * */
extern void	ets_timer_init(void);

/* Name: ets_timer_setfn
 * Address: 0x40002c48
 * Description: helper that sets the callback and the argument of a timer.
 * Returns the timer (arg0).
 * */
extern ets_timer_t*	ets_timer_setfn(ets_timer_t* timer, void (*timer_cb)(void*), void* arg);

/* Name: ets_uart_printf
 * Address: 0x40002544
 * Description: wrapper around ets_vprintf.
 * Returns the number of chars printed.
 * */
extern s32	ets_uart_printf(const char* fmt, ...);

/* Name: ets_update_cpu_frequency
 * Address: 0x40002f04
 * Description: sets the register at 0x3fffc704.
 * frequency should be in MHz.
 * Returns frequency.
 * */
extern u32	ets_update_cpu_frequency(u32 frequency);

/* Name: ets_vprintf
 * Address: 0x40001f00
 * Description: prints a formated string. Seems to take a maximum of 5 extra arguments.
 * Seems to not set up the uart beforehand.
 * Main printf function where all the hardcore stuff happens.
 * */
extern s32	ets_vprintf(const char* fmt, ...);

/* Name: ets_wdt_disable
 * Address: 0x400030f0
 * Description: disables the watchdog.
 * Clears the hardware watchdog registers.
 * Updates its state.
 * Optionally disarms the system timer depending on the watchdog mode.
 * Returns the current watchdog state from the register at 0x3fffc708.
 * */
extern u32	ets_wdt_disable(void);

/* Name: ets_wdt_enable
 * Address: 0x40002fa0
 * Description: enables the watchdog. 
 * Sets the hardware watchdog registers.
 * Updates its state.
 * Optionally arms the system timer depending on the watchdog mode.
 * Returns the current watchdog state from the register at 0x3fffc708.
 * */
extern u32	ets_wdt_enable(u32 mode, u32 timeout, u32 flags);

/* Name: ets_wdt_get_mode
 * Address: 0x40002f34
 * Description: gets the current watchdog state from the register at 0x3fffc708.
 * */
extern u32	ets_wdt_get_mode(void);

/* Name: ets_wdt_init
 * Address: 0x40003170
 * Description: initialize the watchdog.
 * Disables the watchdog by clearing bit 0 at register 0x60000900.
 * Sets an ISR handler for interrupt 8 via ets_isr_attach
 * Re-enables the watchdog by setting the watchdog bit at register 0x3ff00004.
 * */
extern void	ets_wdt_init(void);

/* Name: ets_wdt_restore
 * Address: 0x40003158
 * Description: restores watchdog to a saved_state.
 * Loads saved configuration from 0x3fffc70c and 0x3fffc710.
 * Calls ets_wdt_enable.
 * If saved_state == 0 then does nothing.
 * */
extern void	ets_wdt_restore(u32 saved_state);

/* Name: ets_write_char
 * Address: 0x40001da0
 * Description: writes a char to active uart.
 * Checks for function pointers stored at 0x3fffdd48 (putc1) and 0x3fffdd4c (putc2).
 * */
extern void	ets_write_char(char c);

/* _XTOS FUNCTIONS*/

/* Name: _xtos_alloca_handler
 * Address: 0x4000dbe0
 * Description: minimal exception handler stub that is triggered with the alloca instruction.
 * Directly returns from exception (rfe).
 * */
extern void	_xtos_alloca_handler(void);

/* Name: _xtos_c_wrapper_handler
 * Address: 0x40000598
 * Description: interrupt stub: saves the stack and calls the C handler depending on the cause.
 * Saves all register in the stack with the following layout but the sp 
 * (which can be found at frame + 256):
 struct frame
 {
 	u32	epc1;
	u32	ps;
	s32	sar;
	u32	rsvd0;
	u32	a0;
	u32	a[14];
 };
 void	interrupt_handler_signature(struct frame* sp, u32 cause);
 * Reads the cause in a2 which is an index into a handler table at 0x3fffc100.
 * If handler is exists, calls it with (stack pointer, cause)
 * Restores SAR jumps to 0x4000dc54 (_xtos_return_from_exc).
 * */
extern void	_xtos_c_wrapper_handler(u32 cause);

/* Name: _xtos_cause3_handler
 * Address: 0x40000590
 * Description: calls _xtos_c_wrapper_handler with 3 as the parameter.
 * */
extern void	_xtos_cause3_handler(void);

/* Name: _xtos_ints_off
 * Address: 0x4000bda4
 * Description: clear the interrupts flags with the mask by writing to the register at 0x3fffc200.
 * Seems to return something but I don't know what.
 * Called by ets_isr_mask.
 * */
extern void	_xtos_ints_off(u32 mask);

/* Name: _xtos_ints_on
 * Address: 0x4000bd84
 * Description: sets the interrupts flags with by writing to the register at 0x3fffc200.
 * Seems to return something but I don't know what.
 * Called by ets_isr_unmask.
 * */
extern void	_xtos_ints_on(u32 mask);

/* Name: _xtos_l1int_handler
 * Address: 0x4000048c
 * Description: level 1 interrupt handler. Handles pending enabled interrupts.
 * Saves CPU context and the current interrupt mask from 0x3fffc204.
 * Iterates through pending enabled interrupts and dispatching each to its registered handler.
 * Restores SAR jumps to 0x4000dc54 (_xtos_return_from_exc).
 * */
extern void	_xtos_l1int_handler(void);

/* Name: _xtos_p_none
 * Address: 0x4000dbf8
 * Description: empty handler (just ret.n).
 * */
extern void	_xtos_p_none(void);

/* Name: _xtos_restore_intlevel
 * Address: 0x4000056c
 * Description: restores interrupt level by writing to special register ps arg0. 
 * */
extern void	_xtos_restore_intlevel(u32 interrupt_level);

/* Name: _xtos_return_from_exc
 * Address: 0x4000dc54
 * Description: restores the CPU context.
 * Restores a0-a15 except a2 and a3.
 * Restores epc1 with a2 and ps with a3.
 * Restores a2 and a3.
 * Restores stack pointer by adding 256 to it.
 * */
extern void	_xtos_return_from_exc(void); 

/* Name: _xtos_set_exception_handler
 * Address: 0x40000454
 * Description: sets an exception handler.
 * cause is used as an index in the handler table at 0x3fffc100 and in the vector table at 0x3fffc000.
 * If handler == 0 then 
 * 	- uses _xtos_unhandled_exception as the default vector.
 * 	- uses _xtos_p_none as the default handler.
 * Else
 * 	- uses _xtos_c_wrapper_handler as the vector.
 * 	- uses handler as the handler.
 * The first argument of the handler is the stack pointer containing the stack frame 
 * described in the _xtos_c_wrapper_handler comment above.
 * The second argument of the handler is the cause.
 * On success: returns the previous handler pointer.
 * On error (if cause <= 64): returns 0.
 * */
extern void*	_xtos_set_exception_handler(u32 cause, void (*handler)(u32* stack_frame, u32 cause);

/* Name: _xtos_set_interrupt_handler
 * Address: 0x4000bd70
 * Description: wrapper around _xtos_set_interrupt_handler_arg.
 * Uses interrupt_nb as argument for the handler.
 * On success: returns previous handler pointer.
 * On error (interrupt_nb > 14 
 * 	|| interrupt_type from 0x3fffd650 + interrupt_nb < 3 
 * 	|| previous_handler_ptr == _xtos_unhandled_interrupt): returns 0.
 * */
extern void*	_xtos_set_interrupt_handler(u32 cause, void (*handler)(u32 _cause));

/* Name: _xtos_set_interrupt_handler_arg
 * Address: 0x4000bd28
 * Description: sets up the interrupt handler with an argument in the interrupt table.
 * at 0x3fffc208 - (interrupt_nb << 3) (grows downward).
 * Called by ets_isr_attach. 
 * If handler == 0 then 
 * 	- uses _xtos_unhandled_interrupt as the default handler.
 * 	- uses interrupt_nb as the default argument.
 * Else
 * 	- uses handler as the handler.
 * 	- uses arg as the argument.
 * On success: returns previous handler pointer.
 * On error (interrupt_nb > 14 
 * 	|| interrupt_type from 0x3fffd650 + interrupt_nb < 3 
 * 	|| previous_handler_ptr == _xtos_unhandled_interrupt): returns 0.
 * */
extern void*	_xtos_set_interrupt_handler_arg(u32 interrupt_nb, void (*handler)(void*), void* arg); 

/* Name: _xtos_set_intlevel
 * Address: 0x4000dbfc
 * Description: sets the interrupt level to new_level.
 * Masks out new_level to extract the 4 most right bits.
 * Level 0: all interrupts are enabled.
 * Level 1-15: Interrupts at level <= current_level are masked.
 * Returns the previous interrupt level.
 * */
extern u32	_xtos_set_intlevel(u32 new_level);

/* Name: _xtos_set_min_intlevel
 * Address: 0x4000dc18
 * Description: sets the minimum interrupt level to min_level if the current level is lower.
 * Returns the previous interrupt level.
 * */
extern u32	_xtos_set_min_intlevel(u32 min_level);

/* Name: _xtos_set_vpri
 * Address: 0x40000574
 * Description: sets the virtual priority interrupt mask.
 * Disable interrupts.
 * Reads previous mask and sets new_mask at 0x3fffc204.
 * Enables new_mask by ANDing base interrupt enable mask at 0x3fffc200 and writing results to INTENABLE.
 * Returns the previous mask.
 * */
extern u32	_xtos_set_vpri(u32 new_mask);

/* Name: _xtos_syscall_handler
 * Address: 0x4000dbe4
 * Description: handles syscalls by treating them as no ops.
 * Returns from interrupt.
 * */
extern void	_xtos_syscall_handler(void);

/* Name: _xtos_unhandled_exception
 * Address: 0x4000dc44
 * Description: default vector exception.
 * Restores registers a2, a3 and sp by adding 256.
 * Triggers a software breakpoint.
 * */
extern void	_xtos_unhandled_exception(void* frame, u32 cause);

/* Name: _xtos_unhandled_interrupt
 * Address: 0x4000dc3c
 * Description: default interrupt handler.
 * Triggers a software breakpoint.
 * */
extern void	_xtos_unhandled_interrupt(void* arg);

/* UART FUNCTIONS */

/* Name: uart_tx_one_char
 * Description: sends a byte over the active debug UART.
 * Seems to find which UART is active by looking if the flag of register 0xfffde50 is set to 1 or not.
 * */
extern void	uart_tx_one_char(u8 c);
extern void	uart_div_modify(u8 uart, u32 div);

/* ROM TABLES */

/* Name: _rom_store
 * Address: 0x4000e388
 * Description: table that seems to contain unreferenced code/data.
 * */
extern u32*	_rom_store;

/* Name: _rom_store_table
 * Address: 0x4000e328
 * Description: table that seems to contain referenced code/data.
 * */
extern u32*	_rom_store_table;

/* ROM FUNCTIONS */

/* Name: phy_get_romfuncs
 * Address: 0x40006b08
 * Description: returns a pointer to the PHY ROM function table stored at 0x3fffc730.
 * Note: PHY stands for Physical Layer and refers to the WiFi components and circuits that:
 * 	- Transmit side:
 * 		Converts digital data into analog radio frequency (RF) signals
 * 		Modulates the carrier wave (OFDM for Wi-Fi)
 * 		Amplifies and transmits through the antenna
 * 	- Receive side:
 * 		Receives RF signals from the antenna
 * 		Amplifies and filters the signal
 * 		Demodulates back to digital data
 * 		Performs timing synchronization and channel estimation
 * Functions are used by the WiFi hardware.
 * */
extern void*	phy_get_romfuncs(void);

/* Name: rom_abs_temp
 * Address: 0x400060c0
 * Description: returns the absolute value a s32.
 * */
extern s32	rom_abs_temp(s32 value);

/* Name: rom_ana_inf_gating_en
 * Address: 0x40006b10
 * Description: configures the analog interface gating for power management.
 * If the interface is enabled:
 * 	- writes config and flags to register 119 by calling a PHY write function at offset 156
 * 	with the following signature:
 * 		- void (*phy_write)(u8 reg, u8 block, u8 field, u8 mask, u8 value);
 * Else:
 * 	- sets basic gating control bits.
 * It seems that config is split in 3 parts: bits 0-3, bits 4-11, bits 12-19.
 * Same for flags: bits 0-7, bits 8-15, bits 16-19.
 * */
extern void	rom_ana_inf_gating_en(u32 enable, u32 config, u32 flags);

/* Name: rom_cal_tos_v50
 * Address: 0x40007a28
 * Description: performs a TX output stage calibration for the WiFi PHY version 5.0.
 * Note: DAC stands for Digital-to-Analog Converter.
 * Uses a binary search algorithm to find the optimal calibration
 * by reading back signal quality indicators from the hardware at register 0x60000d4c.
 * If save_to_reg == true then save calibration to register 0x60009864.
 * Saves values in result: [0] = dac1 and [1] = dac0.
 * Returns save_to_reg (arg0).
 * */
extern bool	rom_cal_tos_v50(bool save_to_reg, u16 _unused, u32 delay_us, u32 iterations, u8* result);

/* Name: rom_chip_50_set_channel
 * Address: 0x40006f84
 * Description: sets the channel for WiFi PHY version 5.0.
 * channel must be between 0-14.
 * Args 1 and 3-6 are passed to the PHY setup function at offset 128.
 * Then, uses PHY functions in order at offsets:
 * 	- 152 params(103,4,7,4) maybe a phy_write function
 * 	- 124 params(1, retval<<8, 0x96000) 
 * 	- then delays.
 * Updates register 0x60009b14 by ORing it with (0x00005272 << 17) and (0x00006000)
 * Then seem to do some operations on the channel.
 * Then updates 0x600098a0 by ANDing it with () and ORing it with (*local0 10 first bits << 20) and (0x000fffff)
 * */
extern void	rom_chip_50_set_channel(
		u8 channel, u16 arg1, bool does_conf, u16 arg3, u16 arg4, u16 arg5, u16 arg6);

/* Name: rom_chip_v5_disable_cca
 * Address: 0x400060d0
 * Description: disables CCA for WiFi PHY version 5.0.
 * Note: CCA stands for Clear Channel Assessment.
 * Note: CSMA/CA stands for Carrier Sense Multiple Access with Collision Avoidance.
 * CCA is used to sense if the channel is used for other transmissions to avoid collisions.
 * Sets bit 28 of PHY register 0x60009b00.
 * */
extern void	rom_chip_v5_disable_cca(void);

/* Name: rom_chip_v5_enable_cca
 * Address: 0x400060ec
 * Description: enables CCA for WiFi PHY version 5.0.
 * Unsets bit 28 of PHY register 0x60009b00.
 * */
extern void	rom_chip_v5_enable_cca(void);

/* Name: rom_chip_v5_rx_init
 * Address: 0x4000711c
 * Description: initialize the receiver (RX) for WiFi PHY version 5.0.
 * Calls 5 times a phy_write function at offset 152 seen previously with rom_chip_50_set_channel
 * with various parameters.
 * */
extern void	rom_chip_v5_rx_init(void);

/* Name: rom_chip_v5_sense_backoff
 * Address: 0x4000610c
 * Description: configures the carrier sense backoff threshold for for WiFi PHY version 5.0.
 * Basically adjust the sensitivity of the CSMA/CA.
 * */
extern void	rom_chip_v5_sense_backoff(s8 backoff_value);

/* Name: rom_chip_v5_tx_init
 * Address: 0x4000718c
 * Description: initialize the transmiter (TX) for WiFi PHY version 5.0.
 * Sets up the transmiter in a similar manner the receiver is initialized.
 * */
extern void	rom_chip_v5_tx_init(void);

/* Name: rom_dc_iq_est
 * Address: 0x4000615c
 * Description: performs DC offset and IQ imbalance estimation for the receiver.
 * Note: DC stands for Direct Current.
 * Note: IQ stands for In-phase and Quadrature.
 * Note: ADC stands for Analog to Digital Converter.
 * Used to calibrated the hardware before receiving any signal.
 * arg0 is used in the PHY function at offset 52 from the function table as the first argument.
 * results[0] = In-phase channel DC offset.
 * results[1] = Quadrature channel DC offset.
 * */
extern void	rom_dc_iq_est(s32 arg0, s32 samples, s32* results);

/* Name: rom_en_pwdet
 * Address: 0x400061b8
 * Description: enables / configure the power detector for the transmiter.
 * Calls PHY function at offset 76 with mode as a parameter.
 * Clears bit 21 and 23 of register 0x60000d5c.
 * mode == 0 disables power detector.
 * mode == 1 enables power detector.
 * mode != 0 || mode != 1 just does the setup call and bit clearing.
 * Mesures of pwdet can be used for:
 * 	- TX power calibration (actual vs programmed power)
 * 	- power control (adjust Power Amplifier)
 * 	- regulatory complience (no excess)
 * 	- temperature compensation (temperature changes output power)
 * 	- linearity optimization (avoid PA compression)
 * */
extern void	rom_en_pwdet(u32 mode);

/* Name: rom_get_bb_atten
 * Address: 0x40006238
 * Description: calculates baseband attenuation needed to reach a target value for the transmiter.
 * Note: Baseband attenuation controls signal level in the digital domain before the DAC.
 * Note: DAC stands for	Digital to Analog Converter.
 * adjustment = (target + 8) - (current * 4)
 * If adjustment < 0 then adjustment = 0.
 * If adjustment > 127 then adjustment = 127.
 * Returns the attenuation adjustment clamped to the range 0-127.
 * */
extern s8	rom_get_bb_atten(s16 target, s16 current);

/* Name: rom_get_corr_power
 * Address: 0x40006260
 * Description: calculates correlation power metrics from PHY accumulator registers.
 * Computes signal power, correlation power and DC power for receiver signal quality assessment.
 * Function typically called during packet reception or channel assessment 
 * to monitor signal quality and trigger calibration/adjustment if needed.
 * dc_I >> (shift - 2) read from 0x600005dc
 * dc_Q >> (shift - 2) read from 0x600005e0
 * dc_pwr >> (shift - 2) read from 0x600005e4
 * corr_I = corr_I1 + corr_Q2
 * corr_Q = corr_Q1 - corr_I2
 * corr_pwr = ((corr_I)² + (corr_Q)²) >> (shift * 2 - 22)
 * dc_pwr_offset = (dc_I)² + (dc_Q)²
 * results[0] = dc_pwr
 * results[1] = corr_pwr
 * results[2] = dc_pwr_offset
 * */
extern void	rom_get_corr_power(s32* results, u32 shift);

/* Name: rom_get_fm_sar_dout
 * Address: 0x400062dc
 * Description: reads FM SAR ADC outputs, processes 8 samples
 * and computes averaged I/Q values with specific weighting (receiver function).
 * Note: FM stands for frequence mismatch.
 * Note: SAR stands for Successive Approximation Register.
 * Note: CORDIC stands for COordinate Rotation DIgital Computer. Efficiently calculates trigonometric functions, rotations, and magnitude/phase using only shifts and adds (no multipliers).
 * Note: DFT stands for Discrete Fourier Transform. Converts time-domain samples into frequency-domain components
 * Read the u32 8 values starting at 0x60000b80 and stores them in a temporary samples[8].
 * Does some weight calculus so compute the I/Q:
 * 	- I = samples[2] + (2 * samples[0]) - (3 * (samples[4] + samples[6]))
 * 	- Q = (2 * (samples[1] + samples[3])) - (samples[4] + samples[6])
 * The specific weighting pattern (2, 1, 1, -3, -3) for samples
 * suggests this implements a known frequency estimation algorithm,
 * possibly related to CORDIC or DFT-based frequency detection.
 * */
extern void	rom_get_fm_sar_dout(s16* out_i, s16* out_q);

/* Name: rom_get_noisefloor
 * Address: 0x40006394
 * Description: retruns the noisefloor.
 * Reads value from 0x60009b64 and extract bits 20-31.
 * Returns (value >> 3) - 512 as a signed value.
 * */
extern s16	rom_get_noisefloor(void);

/* Name: rom_get_power_db
 * Address: 0x400063b0
 * Description: calculates signal power in dB units by reading correlation power metrics,
 * converting them to dB scale, and applying an offset correction.
 * */
extern s16	rom_get_power_db(s16 offset);

extern void	rom_i2c_writeReg(u8 slave_addr, u8 reg_addr_len, u8 data_len, u32 data);

/* 'STD' FUNCTIONS */
extern int	rand(void);
extern void	srand(u32 seed);

/* Name: strcmp
 * Description: compares two strings.
 * Returns 0 if s1 and s2 are equal, 1 if the different char of s1 > than char of s2 else -1.
 * */
extern s32	strcmp(const char* s1, const char* s2);

/* Name: strcpy
 * Description: copies string src into string dst.
 * Returns dst.
 * */
extern char*	strcpy(char* dst, const char* src);

/* Name: strlen
 * Description: gives the lenght of a string by searching for the '\0' char.
 * */
extern u32	strlen(const char* s);

/* Name: strncmp
 * Description: compares n chars of two strings.
 * Returns 0 if s1 and s2 are equal, 1 if the different char of s1 > than char of s2 else -1.
 * */
extern s32	strncmp(const char* s1, const char* s2, u32 n);	

/* Name: strncpy
 * Description: copies n chars of string src into string dst.
 * Returns dst.
 * */
extern char*	strncpy(char* dst, const char* src, u32 n);

/* Name: strstr
 * Description: search the string needle in the haystack.
 * On success: returns the address of the first char of the needle in the haystack.
 * On error: returns 0.
 * */
extern char*	strstr(const char* haystack, const char* needle);

/* MEMORY FUNCTIONS */

/* Name: mem_calloc
 * Address: 0x40001c2c
 * Description: allocates n_elements of element_size bytes on the heap.
 * Uses ets_memset to set each byte to 0.
 * On success: returns a pointer to the newly allocated memory.
 * On error: returns 0.
 * */
extern void*	mem_calloc(u32 n_elements, u32, element_size);

/* Name: mem_malloc
 * Address: 0x40001b40
 * Description: allocates size bytes on the heap.
 * On success: returns a pointer to the newly allocated memory.
 * On error: returns 0.
 * */
extern void*	mem_malloc(u32 size);

/* Name: mem_free
 * Address: 0x400019e0
 * Description: frees a block of memory allocated on the heap.
 * */
extern void	mem_free(void* p);

/* Name: mem_realloc
 * Address: 0x40001c6c
 * Description: allocates size bytes and copies size bytes of p to the new memory.
 * Frees the previously allocated p.
 * On success: returns a pointer to the newly allocated memory.
 * On error: returns 0.
 * */
extern void*	mem_realloc(void* p, u32 size);

/* Name: mem_init
 * Address: 0x40001998
 * Description: initialize a 4kB heap region handled by a boundery tag allocator.
 * Base is 4 bytes aligned.
 * Header is 4 bytes long and starts at aligned base.
 * struct header
 * {
 *	u16	block_size;	// set to 0x1000
 *	u16	rsvd;
 * };
 * Footer seems to be 5 bytes long and starts at offset 0xf84 from aligned base.
 * struct footer
 * {
 *	u16	block_size;		// set to 0x1000
 *	u16	previous_block_size;	// Set to 0x1000
 *	u8	is_allocated;		// Set to 1
 * };
 * Informations of the heap seem to be saved as:
 * 	- *0x3fffdd30 = start
 * 	- *0x3fffdd34 = end
 * 	- *0x3fffdd38 = high water mark (0x1000)
 * Returns heap_start.
 * */
extern void*	mem_init(void* heap_start);

/* Name: mem_trim
 * Address: 0x40001a14
 * Description: Attempts to shrink an allocated block to a new size,
 * potentially freeing the excess memory.
 * Aligns up new_size to a 4-bytes boundery. Size cannot exceed 0x1000.
 * ptr must be in the heap bounds.
 * ptr must be the base of a previously allocated block.
 * If space insufficient or sizes are equal does nothing.
 * If next block is free:
 * 	- merges the excedent with next block.
 * 	- updates free block header with new combined size.
 * 	- updates current block header with reduced size.
 * Else:
 * 	- splits if there is space for it. Minimum is 20 bytes (header and minimum data size).
 * On success: returns arg0.
 * On error: returns 0.
 * */
extern void*	mem_trim(void* ptr, u32 new_size); 

/* Name: mem_zalloc
 * Address: 0x40001c58
 * Description: wrapper around mem_calloc. Zeros out allocated memory.
 * On success: returns the allocated memory.
 * On error: returns 0.
 * */
extern void*	mem_zalloc(u32 size); 

/* Name: memcmp
 * Address: 0x4000dea8
 * Description:
 * */
extern s32	memcmp(const void* b1, const void* b2, u32 n);

/* Name: memcpy
 * Address: 0x4000df48
 * Description: compares n bytes of b1 and b2.
 * Returns 0 if b1 and b2 are equal, 1 if the different byte of b1 > than byte of b2 else -1.
 * */
extern void*	memcpy(void* dst, const void* src, u32 n);

/* Name: memmove
 * Address: 0x4000e04c
 * Description: copies n bytes from src to dst. Handles overlapping cases of dst and src.
 * Returns dst.
 * */
extern void*	memmove(void* dst, const void* src, u32);

/* Name: memset
 * Address: 0x4000e190
 * Description: sets n bytes of s with c.
 * Returns s.
 * */
extern void*	memset(void* s, s32 c, u32 n);

#endif
