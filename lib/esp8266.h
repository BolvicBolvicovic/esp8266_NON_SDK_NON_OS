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
 * Description: wrapper around bzero.
 * Zeros n bytes of array s.
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

/* Name: rom_i2c_readReg
 * Address: 0x40007268
 * Description: reads a single byte from the I2C register for a slave device.
 * Note: I2C stands for Inter-Intergrated Circuit. The I2C bus controls all connected devices to the MCU.
 * Base address of the I2C controller is 0x60000a00 + i2c_num * 4 + 0x300.
 * u32 command_word = (reg_addr << 8) | slave_addr;
 * Writes command_word and polls bit 25 of that register until the byte can be read from bits 16-23.
 * */
extern u8	rom_i2c_readReg(u8 i2c_num, u8 reg_addr, u8 slave_addr);

/* Name: rom_i2c_readReg_Mask
 * Address: 0x4000729c
 * Description: reads bit_length bits from bit_start of the desired I2C register.
 * Calls a function at 0x3fffc730 + 0x90 that takes the 3 first args as arguments to get the full mask.
 * Masked extracted value is 8 bits long.
 * */
extern u8	rom_i2c_readReg_Mask(u8 i2c_num, u8 reg_addr, u8 slave_addr, u8 bit_start, u8 bit_length);

/* Name: rom_i2c_writeReg
 * Address: 0x400072d8
 * Description: writes a single byte to the I2C register for a slave device.
 * Sets bit 24 of command_word to signify a write command and writes value to bits 16-23.
 * */
extern void	rom_i2c_writeReg(u8 slave_addr, u8 i2c_num, u8 reg_addr, u8 value);

/* Name: rom_i2c_writeReg_Mask
 * Address: 0x4000730c
 * Description: writes bit_length bits of value from bit_start to the desired I2C register.
 * Calls a function at 0x3fffc730 + 0x90 that takes the 3 first args as arguments to get the full mask.
 * Calls a function at 0x3fffc730 + 0x98 to write updated mask.
 * */
extern void	rom_i2c_writeReg_Mask(
		u8 slave_addr, u8 i2c_num, u8 reg_addr, u8 bit_start, u8 bit_length, u8 value); 

/* Name: rom_iq_est_disable
 * Address: 0x40006400
 * Description: disables the iq estimation feature.
 * Control register is at 0x6000057c.
 * Reads from register, clears bits with mask 0xfffa0001 and sets bit 12, writes to register,
 * effectively clearing the previous configuration.
 * Reads from register, clears bit 0, writes to register, effectively disabling the feature.
 * */
extern void	rom_iq_est_disable(void);

/* Name: rom_iq_est_enable
 * Address: 0x40006430
 * Description: enables the iq estimation feature.
 * Bit 1 is set, effectively enabling the feature.
 * The two arguments are used to configure the feature by putting them in a command word 
 * at bits 31-18 and 16-2 but what they are per say is not clear.
 * */
extern void	rom_iq_est_enable(u8 arg0, u16 arg1);

/* Name: rom_linear_to_db
 * Address: 0x40006484
 * Description: converts a linear power to decibels.
 * Copies to the stack a table from 0x3fffcbd0.
 * Normalizes the input using 3 as the threshold before calculating the dB value.
 * */
extern s16	rom_linear_to_db(u32 linear_value, u8 scalar);

/* Name: rom_mhz2ieee
 * Address: 0x400065a4
 * Description: converts a frequency in MHz to an IEEE 802.11 channel number.
 * Note: IEEE stands for Institute of Electrical and Electronics Engineers.
 * IEEE 802.11 is specifically the WiFi ranges standard designed by this institute.
 * If bit 7 of flags is set (any bandwidth) then:
 * 	- if freq == 2484 then return 14
 * 	- if freq > 2483 then return (freq - 5000)) / 20 + 15 (5 GHz channels)
 * 	- if freq <= 2483 then return (freq - 2407) / 5 (2.4 GHz channels)
 * Else:
 * 	- if freq == 2484 then return 14
 * 	- if freq > 2483 then return -1 (invalid)
 * 	- if freq <= 2483 then return (freq - 2407) / 5 (2.4 GHz channels)
 * */
extern s8	rom_mhz2ieee(u32 freq, u8 flags);

/* Name: rom_pbus_dco___SA2
 * Address: 0x40007bf0
 * Description: performs an I2C-based DCO calibration on a peripherical bus device.
 * Note: DCO stands for Digitally Controlled Oscillator.
 * The source of configurations comes from the I2C function call located at 0x3fffc730 + 0xb0.
 * DAC is adjusted with a binary search-like algorithm.
 * Uses multiple configurations and runs the adjustments 12 times per configuration.
 * The number of configurations comes from the amount of bits set
 * in the bit segment 6-2 of the value issued by the source.
 * Variation in the configurations comes from specific bit patterns (4-2) in the value saved at sp + 84.
 * Calls eprintf for debug and verbose.
 * */
extern void	rom_pbus_dco___SA2(
		u8 slave_addr, u8 i2c_num, u8 reg_addr, u32 delay_us, bool debug_print, bool verbose);

/* Name: rom_pbus_debugmode
 * Address: 0x4000737c
 * Description: conditionaly sets up and enables debug mode for the PBUS system.
 * PBUS stands for peripherical bus.
 * Waits for the hardware to be ready before enabling the debug mode.
 * */
extern void	rom_pbus_debugmode(void);

/* Name: rom_pbus_enter_debugmode
 * Address: 0x40007410
 * Description: PBUS enters debug mode.
 * arg is passed onto 3 functions calls from the table at 0x3fffc730 (offsets 0xa0, 0xc0, and 0xd0).
 * */
extern void	rom_pbus_enter_debugmode(u32 arg);

/* Name: rom_pbus_exit_debugmode
 * Address: 0x40007448
 * Description: PBUS exits debug mode.
 * Does 3 functions calls from the table at 0x3fffc730 (offsets 0xc8, 0xc4, and 0xbc).
 * */
extern void	rom_pbus_exit_debugmode(void);

/* Name: rom_pbus_force_test
 * Address: 0x4000747c
 * Description: forces the PBUS to run a test operation.
 * Builds a control word with the arguments.
 * Applies control word to register 0x60000594 and waits until the completion of the test
 * by polling bit 1 of register 0x600005a0.
 * */
extern void	rom_pbus_force_test(u32 channel, u32 command, u32 flags);

/* Name: rom_pbus_rd
 * Address: 0x400074d8
 * Description: reads a field of a channel of the PBUS
 * Gets channel byte n with a lookup on table at 0x3fffcce0 using channel_idx
 * and combines it with the field_idx.
 * Returns the low 9 bits of the value read at 0x600005a4 + 4 * (n / 3).
 * */
extern u16	rom_pbus_rd(u32 channel_idx, u32 field_idx);

/* Name: rom_pbus_set_rxgain
 * Address: 0x4000754c
 * Description: sets up the RF receive gain configuration into PBUS.
 * Note: RF stands for Radio Frequency.
 * Note: LNA stands for Low-Noise Amplifier.
 * Note: IF stands for Intermediate Frequency.
 * Decodes the packed rxgain configuration word into several PBUS registers
 * controlling LNA/Mixer/IF‑gain, enabling and tuning the receive gain path.
 * Uses function table at 0x3fffc730 (offsets 0xb0, 0xac).
 * */
extern void	rom_pbus_set_rxgain(u16 rxgain);

/* Name: rom_pbus_set_txgain
 * Address: 0x40007610
 * Description: sets up the RF transmit gain configuration into PBUS.
 * Decodes the packed txgain configuration word and writes the corresponding PBUS registers 
 * that control the transmit gain chain.
 * Uses the function at 0x3fffc730 + 0xac.
 * */
extern void	rom_pbus_set_txgain(u16 txgain);

/* Name: rom_pbus_workmode
 * Address: 0x40007648
 * Description: switch off the debug mode bit of the RF/PBUS block back to work mode.
 * Clears bit 1 in the register at 0x60000594.
 * Clears bit 27 in the register at 0x60009b08.
 * Complete transition back to work mode with function call at 0x3fffc730 + 0x78.
 * */
extern void	rom_pbus_workmode(void);

/* Name: rom_pbus_xpd_rx_off
 * Address: 0x40007688
 * Description: turns off RX XPD for one PBUS block.
 * Note: XPD might stand for eXternal Power Down or eXecute Power Down.
 * 3 calls to function at 0x3fffc730 + 0xac with:
 * 	- (2, 1, arg)
 * 	- (3, 1, 0)
 * 	- (3, 2, 0)
 * */
extern void	rom_pbus_xpd_rx_off(u16 arg);

/* Name: rom_pbus_xpd_rx_on
 * Address: 0x400076cc
 * Description: turns on RX XPD for one PBUS block.
 * 2 calls to function at 0x3fffc730 + 0xac with:
 * 	- (2, 1, 388)
 * 	- (3, 2, 6)
 * */
extern void	rom_pbus_xpd_rx_on(void);

/* Name: rom_pbus_xpd_tx_off
 * Address: 0x400076fc
 * Description: turns off TX XPD for one PBUS block.
 * 3 calls to function at 0x3fffc730 + 0xac with:
 * 	- (6, 1, 0)
 * 	- (1, 1 ,12)
 * 	- (2, 1, 0)
 * */
extern void	rom_pbus_xpd_tx_off(void);

/* Name: rom_pbus_xpd_tx_on
 * Address: 0x40007740
 * Description: turns on TX XPD for one PBUS block.
 * 5 calls to function at 0x3fffc730 + 0xac with:
 * 	- (2, 1, 1)
 * 	- (7, 1, 0x5f)
 * 	- (0, 1, arg)
 * 	- (1, 1, 0x7f)
 * 	- (6, 1, 0x7f)
 * */
extern void	rom_pbus_xpd_tx_on(u16 arg);

/* Name: rom_pbus_xpd_tx_on__low_gain
 * Address: 0x400077a0
 * Description: variant of rom_pbus_xpd_tx_on that enbles the low-gain functionality.
 * Does the same 5 calls to function at 0x3fffc730 + 0xac as rom_pbus_xpd_tx_on
 * and adds a 6th call after the second one with (7, 1, 0).
 * */
extern void	rom_pbus_xpd_tx_on__low_gain(u16 arg);

/* Name: rom_phy_reset_req
 * Address: 0x40007804
 * Description: clears the reset request related bits of a PHY register 
 * at 0x60000600 + 0x110 with mask 0xc3ffffff.
 * */
extern void	rom_phy_reset_req(void);

/* Name: rom_restart_cal
 * Address: 0x4000781c
 * Description: restarts the RF PHY calibration procedure.
 * 3 calls to function at 0x3fffc730 + 0x98 with:
 * 	- (98, 1, 0, 0x5f)
 * 	- (98, 1, 0, 0x7f)
 * 	- (98, 1, 0, 0x3f)
 * */
extern void	rom_restart_cal(void);

/* Name: rom_rfcal_pwrctrl
 * Address: 0x40007eb4
 * Description: does the RF calibration of the transmit power control path.
 * Execute the calibration in debug mode.
 * For each entry of input_table (meaning, input_count):
 * 	- Programs some PA / TX‑power–related setting via ROM helpers and PBUS.
 * 	- Measures a corresponding RF result using measurement functions from table at 0x3fffc730
 * 		([[+0x2c]], [[+0x50]], [[+0x68]] etc.).
 *	- Compares / adjusts vs. a reference and repeats a few times if needed.
 *	- Writes per‑entry results or status bytes to an output buffer you provide.
 * Note that the input_table and the output_table must be of the same size.
 * */
extern void	rom_rfcal_pwrctrl(
		u32 rf_context, const u8* input_table, u32 input_count, s32 reference,
		u32 calibration_idx, u8* output_table);

/* Name: rom_rfcal_rxiq
 * Address: 0x4000804c
 * Description: calculates the RF receive I/Q calibration and yields the correction values.
 * Stores result in:
 * 	- output_iq[0] = i_correction;
 * 	- output_iq[1] = q_correction;
 * */
extern void	rom_rfcal_rxiq(
		u32 rf_context, u32 cfg1, u32 cfg2, u8* output_iq, u32 setup_word, u32 aux);

/* Name: rom_rfcal_rxiq_set_reg
 * Address: 0x40008264
 * Description: sets RF receive IQ correction register(s) for one IQ correction value.
 * Bounds the value between -31 included and 31 included.
 * Returns the final saturated correction code.
 * 2 modes:
 * 	- 0: correction is coarse (splits correction and uses different register fields).
 * 	- 1: correction is fine (uses a single register field).
 * If mask != 0 then mask is used with instructions bnone/bany effectively controlling the value's sign.
 * If mask == 0 then no hardware write.
 * */
extern s32	rom_rfcal_rxiq_set_reg(s32 input_correction, u8* status_output, u32 mode, u32 mask);

/* Name: rom_rfcal_txcap
 * Address: 0x40008388
 * Description: conditionally does a 3-stage search for the best RF transmit capacitance settings,
 * stores them and (re)apply them.
 * If word pointed by rf_cfg has capacitor calibrated bit set (18):
 * 	- the calibration word must be stored in cap_word before the function call.
 * 	- fast-path: routine just resores TX-CAP trims from that calibration word.
 * Else performs the TX capacitor calibration, stores it in cap_word and then run the restore routine.
 * Each stage scans 0 to N TX capacitor codes and looks for the code that produces 
 * the largest measurement response (strongest signal or best match).
 * Marks rf_cfg as calibrated by ORing *rf_cfg with 0x00040000, setting bit 18.
 * */
extern void	rom_rfcal_txcap(
		u32* rf_cfg, u32* cap_word, u32 cap_index, u32 ctx,
		u32 measurement_cfg, u32 measurement_delay);

/* Name: rom_rfcal_txiq
 * Address: 0x40008610
 * Description: conditionally calibrate the RF transmit IQ imbalance.
 * If IQ calibrated bit is set (17) take the fast-path with the calibration word in txiq_word.
 * Else 
 * */
extern void	rom_rfcal_txiq(
		u32* rf_cfg, u16* txiq_word, u32 chan_index, u32 ctx,
		u32 measurement_cfg, u32 measurement_delay);

/* Name: rom_rfcal_txiq_cover
 * Address: 0x400088b8
 * Description: wrapper around rom_rfcal_txiq. Adds a layer of test / coverage / refinement.
 * Returns a signed correction.
 * */
extern s8	rom_rfcal_txiq_cover(u8 idx, u32 cfg, u8 mode, u32 pattern, bool verbose, u8 count);

/* Name: rom_rfcal_txiq_set_reg
 * Address: 0x40008a70
 * Description: sets the appropriate RF transmit IQ fields via 0x3fffc730 + 0x9c.
 * Returns the final applied correction after saturation and possible sign flip.
 * */
extern s32	rom_rfcal_txiq_set_reg(s32 correction, s32 fine, u32 mask);

/* Name: rom_rfpll_reset
 * Address: 0x40007868
 * Description: resets the RF PLL.
 * Note: PLL stands for Phase-Locked Loop. An electronic control system that locks a VCO
 * to a reference clock. In RF chips, it generates the precise LO frequency used for transmit and receive.
 * Note: VCO stands for Voltage-Controlled Oscillator.
 * Note: LO stands for Local Oscillator.
 * 5 calls to 0x3fffc730 + 0x98:
 * 	- (98, 1, 10, 0xa6)
 * 	- (98, 1, 10, 0xa7)
 * 	- (98, 1, 10, 0xa5)
 * 	- (99, 0,  1, 0xf3)
 * 	- (98, 1, 11, 0xc0)
 * */
extern void	rom_rfpll_reset(void);

/* Name: rom_rfpll_set_freq
 * Address: 0x40007968
 * Description: sets the RF PLL to the desired frequency.
 * Writes normalized fixed-point fraction to the 3-bytes output buffer and into PLL registers.
 * */
extern void	rom_rfpll_set_freq(u32 whole_freq, u32 mode, u32 frac_high_bit, u8* output);

/* Name: rom_rxiq_cover_mg_mp
 * Address: 0x40008b6c
 * Description: coverage / refinement helper for the RF receive IQ.
 * Note: MG/MP stands for Multi-Gain/Multi-Phase refinement.
 * Runs the receive IQ measurement engine 0x3fffc730 (offsets: 0xF8, 0x48, 0x30, 0x34).
 * Applies some processing to the measured IQ deltas.
 * Writes the I and Q result to their respective buffer.
 * */
extern void	rom_rxiq_cover_mg_mp(
		u32 shift, u32 cfg, u32 gain_idx, u8 ref_q, u8 ref_i, s8* out_i, s8* out_q);

/* Name: rom_rxiq_get_mis
 * Address: 0x40006628
 * Description: computes the receive IQ mismatch.
 * shift is reduced by 2 and used as the barrel‑shifter amount for four MAC registers.
 * Writes results to the 2 bytes out_buffer:
 * 	- out_buffer[0] = normalized cross‑component (phase/quadrature error)
 * 	- out_buffer[1] = normalized in‑phase/amplitude/gain error
 * */
extern void	rom_rxiq_get_mis(
		s32 shift, s32 debug_flag, s32 debug_mask,
		s8 ref_i, s8 ref_q, u8* out_buffer, bool verbose);

/* Name: rom_sar_init
 * Address: 0x40006738
 * Description: initialize the SAR ADC block through one MMIO write and two PBUS/RF configuration writes.
 * Note: MMIO stands for Memory-Mapped Input/Output, essentially
 * some mapped hardware control/status register.
 * */
extern void	rom_sar_init(void);

/* Name: rom_set_ana_inf_tx_scale
 * Address: 0x4000678c
 * Description:
 * */

/* Name: rom_set_channel_freq
 * Address: 0x40006c50
 * Description:
 * */

/* Name: rom_set_loopback_gain
 * Address: 0x400067c8
 * Description:
 * */

/* Name: rom_set_noise_floor
 * Address: 0x40006830
 * Description:
 * */

/* Name: rom_set_rxclk_en
 * Address: 0x40006550
 * Description:
 * */

/* Name: rom_set_txbb_atten
 * Address: 0x40008c6c
 * Description:
 * */

/* Name: rom_set_txclk_en
 * Address: 0x4000650c
 * Description:
 * */

/* Name: rom_set_txiq_cal
 * Address: 0x40008d34
 * Description:
 * */

/* 'STD' FUNCTIONS */

/* Name: bzero
 * Address: 0x4000de84
 * Description: zeros n bytes of array s.
 * */
extern void	bzero(void* s, u32 n);

/* Name: cmd_parse
 * Address: 0x40000814
 * Description: 
 * */

/* Name: conv_str_decimal
 * Address: 0x40000b24
 * Description: converts a string to a decimal value.
 * */
extern s32	conv_str_decimal(const char* str);

/* Name: conv_str_hex
 * Address: 0x40000b24
 * Description: converts a string to a hexadecimal value.
 * */
extern s32	conv_str_hex(const char* str);

/* Name: convert_para_str
 * Address: 0x40000a60
 * Description: 
 * */

/* Name: eprintf
 * Address: 0x40001d14
 * Description: 
 * */

/* Name: eprintf_init_buf
 * Address: 0x40001cb8
 * Description:
 * */

/* Name: eprintf_to_host
 * Address: 0x40001d48
 * Description:
 * */

/* Name: est_get_printf_buf_remain_len
 * Address: 0x40002494
 * Description:
 * */

/* Name: est_reset_printf_buf_len
 * Address: 0x4000249c
 * Description:
 * */

/* Name: rand
 * Address: 0x40000600
 * Description: returns a semi-random number.
 * Takes values at 
 * 	- a2 = *(0x3fffc6f8 + 168)
 * 	- a3 = *(0x3fffc6f8 + 172)
 * Calls __muldi3 with these values.
 * Updates *(0x3fffc6f8 + 172).
 * ANDs the result with 0x7fffffff.
 * */
extern s32	rand(void);

/* Name: srand
 * Address: 0x400005f0
 * Description: seeds the randomizer.
 * The seed is at 0x3fffc6f8 + 168.
 * Also sets 0x3fffc6f8 + 172 to 0.
 * */
extern void	srand(u32 seed);

/* Name: roundup2
 * Address: roundup2
 * Description:
 * */

/* Name: strcmp
 * Address: 0x4000bdc8
 * Description: compares two strings.
 * Returns 0 if s1 and s2 are equal, 1 if the different char of s1 > than char of s2 else -1.
 * */
extern s32	strcmp(const char* s1, const char* s2);

/* Name: strcpy
 * Address: 0x4000bec8
 * Description: copies string src into string dst.
 * Returns dst.
 * */
extern char*	strcpy(char* dst, const char* src);

/* Name: strlen
 * Address: 0x4000bf4c
 * Description: gives the lenght of a string by searching for the '\0' char.
 * */
extern u32	strlen(const char* s);

/* Name: strncmp
 * Address: 0x4000bfa8
 * Description: compares n chars of two strings.
 * Returns 0 if s1 and s2 are equal, 1 if the different char of s1 > than char of s2 else -1.
 * */
extern s32	strncmp(const char* s1, const char* s2, u32 n);	

/* Name: strncpy
 * Address: 0x4000c0a0
 * Description: copies n chars of string src into string dst.
 * Returns dst.
 * */
extern char*	strncpy(char* dst, const char* src, u32 n);

/* Name: strstr
 * Address: 0x4000e1e0
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
