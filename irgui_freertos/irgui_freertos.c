#include <stdio.h>
#include <string.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include <FreeRTOS.h>
#include <task.h>

#include "application_freertos_prio.h"
#include "gui.h"
#include "ir_nec.h"
#include "uart.h"
#include "dwt_delay.h"
#include "ssd1306.h"

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

static void task_led(__attribute__((unused))void * pvParameters)
{
   while (1)
   {
      gpio_toggle(GPIOC, GPIO13);
      vTaskDelay(pdMS_TO_TICKS(500));
   }
}

/////////////////////////////////////////////////////////////

static void task_ir_recv(__attribute__((unused))void * pvParameters)
{
   nec_err_t rc;
   IR_NEC_MSG msg;

   uint32_t recv_cnt = 0;

   while (1)
   {
      recv_cnt++;
      rc = ir_nec_recv(&msg, portMAX_DELAY);
      if (rc == NEC_OK)
      {
         if (msg.wd != IR_NEC_MSG_REPEAT_CODE)
         {
            gui_set_nec_message(msg);
            printf("%lu : addr=0x%02x, cmd=0x%02x, cmd_n=0x%02x\n",
                   recv_cnt, msg.bs.addr, msg.bs.cmd, msg.bs.cmd_inv);
         }
         else
         {
            printf("%lu : repeat\n", recv_cnt);
         }
      }
      else
      {
         printf("%lu : rc=%u\n", recv_cnt, rc);
      }
      fflush(stdout);

      gui_set_nec_count(recv_cnt);
   }
}

/////////////////////////////////////////////////////////////

int main(void)
{
   // initialize hardware
   init_clock();
   init_gpio();
   uart_init();
   ir_nec_init(TASK_IR_DECODE_INT_WRK_PRIO);
   dwt_delay_init();
   ssd1306_init();

   // initialize GUI
   gui_init();

   xTaskCreate(task_led,     "LED",     50, NULL, TASK_LED_PRIO,     NULL);
   xTaskCreate(task_ir_recv, "IRRECV", 300, NULL, TASK_IR_RECV_PRIO, NULL);

   printf("heap-free: %u\n", xPortGetFreeHeapSize());

   vTaskStartScheduler();
   while (1)
   {
      ;
   }

   return 0;
}

/////////////////////////////////////////////////////////////

#if (configASSERT_DEFINED == 1)

void vAssertCalled(unsigned long ulLine, const char * const pcFileName)
{
   taskENTER_CRITICAL();
   {
      printf("*** ASSERT => %s:%lu\n", pcFileName, ulLine);
      fflush(stdout);
   }
   taskEXIT_CRITICAL();

   while (1)
   {
      ;
   }
}
#endif // configASSERT_DEFINED

/////////////////////////////////////////////////////////////

#if (configCHECK_FOR_STACK_OVERFLOW == 1 || configCHECK_FOR_STACK_OVERFLOW == 2)

// Used while developing.
// FreeRTOS will call this function when detecting a stack overflow.
// The parameters could themselves be corrupted, in which case the
// pxCurrentTCB variable can be inspected directly
// Set a breakpoint in this function using the hw-debugger.

void vApplicationStackOverflowHook(TaskHandle_t xTask,
                                   signed char *pcTaskName);

void vApplicationStackOverflowHook(TaskHandle_t xTask,
                                   signed char *pcTaskName)
{
   (void) xTask;
   (void) pcTaskName;

   gpio_set(GPIOC, GPIO13);
   while (1)
   {
      ;
   }
}

#endif // configCHECK_FOR_STACK_OVERFLOW
