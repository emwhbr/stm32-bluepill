#ifndef SHARED_UART_H
#define SHARED_UART_H

void uart_init(void);

void uart_putc(char c);
void uart_printf(const char *fmt, ...);

char uart_getc(void);
char uart_poll(void);

#endif
