#include <stdio.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include <FreeRTOS.h>
#include <task.h>

#include "uart.h"

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

static void task_led(void *args)
{
   (void)args;

   while (1)
   {
      gpio_toggle(GPIOC, GPIO13);
      vTaskDelay(pdMS_TO_TICKS(500));
   }
}

/////////////////////////////////////////////////////////////

static void task_uart(void *args)
{
   (void)args;

   char input_buf[16];

   while (1)
   {
      printf("\nEnter text: ");
      fflush(stdout);
      fgets(input_buf, 16, stdin);

      printf("text = %s", input_buf);
   }
}

/////////////////////////////////////////////////////////////

int main(void)
{
   // initialize hardware
   init_clock();
   init_gpio();
   uart_init();

   printf("\nstdio_freertos - started\n");

   xTaskCreate(task_led,  "LED",  100, NULL, configMAX_PRIORITIES-1, NULL);
   xTaskCreate(task_uart, "UART", 200, NULL, configMAX_PRIORITIES-2, NULL);

   vTaskStartScheduler();
   while(1)
   {
      ;
   }

   return 0;
}
