#include <stdio.h>
#include <string.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include <FreeRTOS.h>
#include <task.h>

#include "uart.h"
#include "dwt_delay.h"
#include "enc28j60.h"

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
      vTaskDelay(pdMS_TO_TICKS(1000));
   }
}

/////////////////////////////////////////////////////////////

static void test_enc28j60_init(void)
{
   enc28j60_init();
}

/////////////////////////////////////////////////////////////

static void print_enc28j60_menu(void)
{
  printf("\n");
  printf("-----------------------------------------\n");
  printf("--          TEST MENU ENC28J60         --\n");
  printf("-----------------------------------------\n");
  printf(" 1. init\n");
  printf("\n");
}

/////////////////////////////////////////////////////////////

static void task_enc28j60(void *args)
{
   (void)args;

   char input_buf[16];
   int value;

   while (1)
   {
      print_enc28j60_menu();
      
      printf("Enter choice : ");
      fflush(stdout);

      fgets(input_buf, 16, stdin);
      sscanf(input_buf, "%d", &value);

      switch (value)
      {
         case 1:
            test_enc28j60_init();
            break;
         default:
            printf("*** Illegal choice : %s\n", input_buf);
      }
   }
}

/////////////////////////////////////////////////////////////

int main(void)
{
   // initialize hardware
   init_clock();
   init_gpio();
   uart_init();
   dwt_delay_init();

   printf("\nenc28j60_freertos - started\n");

   xTaskCreate(task_led,      "LED",      100, NULL, configMAX_PRIORITIES-1, NULL);
   xTaskCreate(task_enc28j60, "ENC28J60", 250, NULL, configMAX_PRIORITIES-2, NULL);

   vTaskStartScheduler();
   while(1)
   {
      ;
   }

   return 0;
}
