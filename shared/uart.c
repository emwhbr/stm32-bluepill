#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>

#include "libopencm3/stm32/rcc.h"
#include "libopencm3/stm32/gpio.h"
#include "libopencm3/stm32/usart.h"
#include <libopencm3/cm3/nvic.h>

#include "../shared/uart.h"

// we are not using interrupt yet
#if 0

/////////////////////////////////////////////////////////////

#define UART_RX_BUF_SIZE  (16)

// circular receiver buffer
struct uart_rx_buffer
{
   volatile uint8_t head; // next index to read from
   volatile uint8_t tail; // next index to write to
   uint8_t data[UART_RX_BUF_SIZE];
};

static struct uart_rx_buffer* volatile rx_buffer = 0;

/////////////////////////////////////////////////////////////

void usart1_isr(void)
{
   char ch;
   uint32_t ntail;
   if (!rx_buffer)
   {
      return;
   }

   while (USART_SR(USART1) & USART_SR_RXNE)
   {
      ch = USART_DR(USART1);
      ntail = (rx_buffer->tail + 1) % UART_RX_BUF_SIZE; // next tail index

      // write to the receive buffer if is not full
      if (ntail != rx_buffer->head)
      {
         rx_buffer->data[rx_buffer->tail] = ch; // write to buffer
         rx_buffer->tail = ntail;               // update tail index
      }
   }
}
#endif

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

   // TX: PA9
   gpio_set_mode(GPIOA,
      GPIO_MODE_OUTPUT_50_MHZ,
      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
      GPIO_USART1_TX);

   // RX : PA10
   gpio_set_mode(GPIOA,
      GPIO_MODE_INPUT,
      GPIO_CNF_INPUT_FLOAT,
      GPIO_USART1_RX);

   // 115200-8-N-1
   usart_set_baudrate(USART1, 115200);
   usart_set_databits(USART1, 8);
   usart_set_stopbits(USART1, USART_STOPBITS_1);
   usart_set_mode(USART1, USART_MODE_TX_RX);
   usart_set_parity(USART1, USART_PARITY_NONE);
   usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

#if 0
   // create the receive buffer
   rx_buffer = malloc(sizeof(struct uart_rx_buffer));
   rx_buffer->head = 0;
   rx_buffer->tail = 0;
   memset(rx_buffer, 0, sizeof(struct uart_rx_buffer));
#endif

#if 0
   // enable interrupt and start USART1
   nvic_enable_irq(NVIC_USART1_IRQ);
   usart_enable_rx_interrupt(USART1);
#endif

   usart_enable(USART1);
}

/////////////////////////////////////////////////////////////

void uart_putc(char c)
{
   // send message via USART
   usart_send_blocking(USART1, c);
}

/////////////////////////////////////////////////////////////

void uart_printf(const char *fmt, ...)
{
	va_list argp;
	va_start(argp, fmt);
	uart_vprint(fmt, argp);
	va_end(argp);
}

/////////////////////////////////////////////////////////////

char uart_getc(void)
{
   while (1)
   {
      if (USART_SR(USART1) & USART_SR_RXNE)
      {
         break;
      }
   }

   return USART_DR(USART1);
}
