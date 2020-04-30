#include <stdio.h>
#include <unistd.h>

#include "uart.h"

/////////////////////////////////////////////////////////////

// The STDIO (libc-nano) functions requires the _read and
// _write functions. Connect these functions to use the UART.
int _write(int fd, char *buf, int size);
int _read(int fd, char *buf, int size);

// circular receive buffer
#define LINE_BUF_SIZE (32)

static uint16_t start_idx = 0;
static uint16_t end_idx = 0;
static char line_buf[LINE_BUF_SIZE + 1];

#define line_buf_len ((end_idx - start_idx) % LINE_BUF_SIZE)
static inline int inc_idx(int n) { return ((n + 1) % LINE_BUF_SIZE); }
static inline int dec_idx(int n) { return (((n + LINE_BUF_SIZE) - 1) % LINE_BUF_SIZE); }

/////////////////////////////////////////////////////////////

static void get_buffered_line(void)
{
   if (start_idx != end_idx)
   {
      return;
   }

   char c;
   while (1)
   {
      c = uart_getc();
      if (c == '\r')
      {
         line_buf[end_idx] = '\n';
         end_idx = inc_idx(end_idx);
         line_buf[end_idx] = '\0';
         uart_putc('\r');
         uart_putc('\n');
         return;
      }
      // NOTE!
      // We dont support editing characters (^H, ^W, ^U or DEL).
      // These characters should be taken care of here.
      else
      {
         // non-editing character so insert it
         if (line_buf_len == (LINE_BUF_SIZE - 2))
         {
            // we have no space left for final \n + \0
            uart_putc('\a');
         }
         else
         {
            line_buf[end_idx] = c;
            end_idx = inc_idx(end_idx);
            uart_putc(c);
         }
      }
   }
}

/////////////////////////////////////////////////////////////

int _write(int fd, char *buf, int size)
{
   if ( ((fd != STDOUT_FILENO) && (fd != STDERR_FILENO)) || !buf )
   {
      return -1;
   }

   int len = 0;
   while (*buf && (len < size))
   {
      uart_putc(*buf);
      if (*buf == '\n')
      {
         uart_putc('\r');
      }
      len++;
      buf++;
   }

   return len;
}

/////////////////////////////////////////////////////////////

int _read(int fd, char *buf, int size)
{
   if ( (fd != STDIN_FILENO) || !buf )
   {
      return -1;
   }

   get_buffered_line();

   int len = 0;

   while ((line_buf_len > 0) && (size > 0))
   {
      *buf++ = line_buf[start_idx];
      start_idx = inc_idx(start_idx);
      len++;
      size--;
   }

   start_idx = 0;
   end_idx = 0;

   return len;
}
