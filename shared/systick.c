#include "libopencm3/cm3/nvic.h"
#include "libopencm3/cm3/systick.h"

#include "systick.h"

volatile uint32_t g_systick_ms = 0;  // millisecond counter, overflow in 49.7 days

/////////////////////////////////////////////////////////////

void sys_tick_handler(void)
{
	// Systick timer reload interrupt handler.
	// Called every time the systick timer reaches its reload value.

	// increment the global millisecond counter
	g_systick_ms++;
}

/////////////////////////////////////////////////////////////

void systick_init(void)
{
	// 72MHz / 8 => 9000_000 counts per second
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);

	// 9000_000 / 9000 = 1000 overflows per second - every 1ms one interrupt
	// SysTick interrupt every N clock pulses: set reload to N-1
	systick_set_reload(8999);
	systick_interrupt_enable();

	// start counting
	systick_counter_enable();
}

/////////////////////////////////////////////////////////////

void systick_delay_ms(uint32_t ms)
{
	// Delay for the specified number of milliseconds.
	// This is implemented by configuring the systick timer to increment
	// a count every millisecond and then busy waiting in a loop.

	uint32_t target = g_systick_ms + ms;
	while (target > g_systick_ms);
}

/////////////////////////////////////////////////////////////

uint32_t systick_get_ms(void)
{
	return g_systick_ms;
}
