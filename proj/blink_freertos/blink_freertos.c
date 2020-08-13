#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include <FreeRTOS.h>
#include <task.h>

#include "uart.h"

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed portCHAR *pcTaskName);

volatile uint32_t g_led_cnt = 0;

/////////////////////////////////////////////////////////////

void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed portCHAR *pcTaskName)
{
   (void)pxTask;
   (void)pcTaskName;
   for(;;);
}

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
      g_led_cnt++;
      gpio_toggle(GPIOC, GPIO13);
      vTaskDelay(pdMS_TO_TICKS(100));
   }
}

/////////////////////////////////////////////////////////////

static void task_uart(void *args)
{
   (void)args;

   UBaseType_t uxHighWaterMark1 = uxTaskGetStackHighWaterMark( NULL );
   UBaseType_t uxHighWaterMark2;

   uint32_t uart_cnt = 0;
   uint32_t tick_cnt = xTaskGetTickCount();
   uint32_t tick_cnt_old = tick_cnt;

   while (1)
   {
      tick_cnt = xTaskGetTickCount();
      uxHighWaterMark2 = uxTaskGetStackHighWaterMark( NULL );

      uart_printf("blink_freertos - led-cnt=%u, uart-cnt=%u, tick_cnt=%u, tick-cnt-delta=%u, stack1=%u, stack2=%u\n",
         g_led_cnt, ++uart_cnt, tick_cnt, tick_cnt - tick_cnt_old, uxHighWaterMark1, uxHighWaterMark2);
      tick_cnt_old = tick_cnt;

      vTaskDelay(pdMS_TO_TICKS(100));
   }
}

/////////////////////////////////////////////////////////////

int main(void)
{
   // initialize hardware
   init_clock();
   init_gpio();
   uart_init();

   // send message on UART
   uart_printf("blink_freertos - started\n");

   xTaskCreate(task_led,  "LED",  100, NULL, configMAX_PRIORITIES-1, NULL);
   xTaskCreate(task_uart, "UART", 200, NULL, configMAX_PRIORITIES-2, NULL);

   vTaskStartScheduler();
   while(1)
   {
      ;
   }

   return 0;
}
