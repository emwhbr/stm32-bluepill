#include <stdio.h>
#include <string.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include <FreeRTOS.h>
#include <task.h>

#include "application_freertos_prio.h"
#include "ir_nec.h"
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

static void task_led(__attribute__((unused))void * pvParameters)
{
   while (1)
   {
      gpio_toggle(GPIOC, GPIO13);
      vTaskDelay(pdMS_TO_TICKS(100));
   }
}

/////////////////////////////////////////////////////////////

static void test_init(void)
{
   ir_nec_init(TASK_IR_DECODE_INT_WRK_PRIO);
}


/////////////////////////////////////////////////////////////

static void test_recv(void)
{
   char input_buf[16];
   unsigned timeout_value;

   uint32_t cnt = 0;
   nec_err_t rc;
   IR_NEC_MSG msg;

   printf("Enter timeout (ms) [0-65535]: ");
   fflush(stdout);

   fgets(input_buf, 16, stdin);
   sscanf(input_buf, "%u", &timeout_value);

   do
   {
      cnt++;
      rc = ir_nec_recv(&msg, pdMS_TO_TICKS(timeout_value));
      if (rc == NEC_OK)
      {
         if (msg.wd != IR_NEC_MSG_REPEAT_CODE)
         {
            printf("%lu : addr=0x%02x, cmd=0x%02x, cmd_n=0x%02x\n",
                   cnt, msg.bs.addr, msg.bs.cmd, msg.bs.cmd_inv);
         }
         else
         {
            printf("%lu : repeat\n", cnt);
         }
      }
      else
      {
         printf("cnt=%lu, rc=%u\n", cnt, rc);
      }
      fflush(stdout);
   } while (rc == NEC_OK);
}

/////////////////////////////////////////////////////////////

static void print_test_menu(void)
{
  printf("\n");
  printf("heap-free: %u\n", xPortGetFreeHeapSize());
  printf("-----------------------------------\n");
  printf("--          TEST MENU            --\n");
  printf("-----------------------------------\n");
  printf(" 1. init\n");
  printf(" 2. recv\n");
  printf("\n");
}

////////////////////////////////////////////////////////////

static void task_test(__attribute__((unused))void * pvParameters)
{
   char input_buf[16];
   int value;

   while (1)
   {
      print_test_menu();

      printf("Enter choice : ");
      fflush(stdout);

      fgets(input_buf, 16, stdin);
      sscanf(input_buf, "%d", &value);

      switch (value)
      {
         case 1:
            test_init();
            break;
         case 2:
            test_recv();
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

   printf("\nirdecode_freertos - started\n");

   // Minium stack size for LED is 50 words (checked with configCHECK_FOR_STACK_OVERFLOW)
   xTaskCreate(task_led,  "LED",   50, NULL, TASK_LED_PRIO,  NULL);
   xTaskCreate(task_test, "TEST", 300, NULL, TASK_TEST_PRIO, NULL);

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
