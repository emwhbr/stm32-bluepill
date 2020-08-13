#include "libopencm3/stm32/rcc.h"
#include "libopencm3/stm32/gpio.h"

#include "uart.h"
#include "systick.h"

/////////////////////////////////////////////////////////////

static void init_clock(void)
{
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	// clock for GPIO port C: USER_LED
	rcc_periph_clock_enable(RCC_GPIOC);
}

/////////////////////////////////////////////////////////////

static void init_gpio(void)
{
	// USER_LED: PC13
	gpio_set_mode(GPIOC,
		GPIO_MODE_OUTPUT_2_MHZ,
		GPIO_CNF_OUTPUT_PUSHPULL,
		GPIO13);
}

/////////////////////////////////////////////////////////////

int main(void)
{
	// initialize hardware
	init_clock();
	init_gpio();

	uart_init();
	systick_init();

	// USER_LED: turn on
	gpio_clear(GPIOC, GPIO13);

	// send message on UART
	uart_printf("blink - started\n");

	uint32_t cnt = 0;
	uint32_t ms = systick_get_ms();
	uint32_t old_ms = ms;

	while(1)
	{
		ms = systick_get_ms();

		gpio_toggle(GPIOC, GPIO13);
		uart_printf("blink - cnt=%u, systick_ms=%u, systick_ms_delta=%u\n",
			cnt++, ms, ms - old_ms);
		old_ms = ms;

		systick_delay_ms(100);
	}
}
