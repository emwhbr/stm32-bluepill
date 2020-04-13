#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "libopencm3/stm32/rcc.h"
#include "libopencm3/stm32/gpio.h"
#include "libopencm3/stm32/usart.h"

#include "../shared/uart.h"

/////////////////////////////////////////////////////////////

static void uart_putc(char c)
{
	// send message via USART
	usart_send_blocking(USART1, c);
}

/////////////////////////////////////////////////////////////

static void uart_puts(const char *str)
{
	if (str != NULL)
	{
		for (size_t i=0; i < strlen(str); i++)
		{
			uart_putc(str[i]);
			if (str[i] == '\n')
			{
				uart_putc('\r');
			}
		}
	}
}

/////////////////////////////////////////////////////////////

static void uart_vprint(const char *fmt, va_list argp)
{
	char str[200];

	if (0 < vsprintf(str, fmt, argp))
	{
		uart_puts(str);
	}
}


/////////////////////////////////////////////////////////////

void uart_init(void)
{
	// clock for GPIO port A: USART1
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_USART1);

	// TX: PA9, RX: PA10 (not used)
	gpio_set_mode(GPIOA,
		GPIO_MODE_OUTPUT_50_MHZ,
		GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
		GPIO_USART1_TX);

	// 115200-8-N-1
	usart_set_baudrate(USART1, 115200);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_mode(USART1, USART_MODE_TX);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	usart_enable(USART1);
}

/////////////////////////////////////////////////////////////

void uart_printf(const char *fmt, ...)
{
	va_list argp;
	va_start(argp, fmt);
	uart_vprint(fmt, argp);
	va_end(argp);
}
