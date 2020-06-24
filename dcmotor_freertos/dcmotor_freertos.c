#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include <FreeRTOS.h>
#include <task.h>

#include "application_freertos_prio.h"
#include "uart.h"
#include "adc.h"
#include "motor.h"
#include "mpc_fsm.h"

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

static void test_read_adc(void)
{
   uint16_t adc_val;

   adc_val = adc_get_value();

   printf("ADC : %04u - 0x%03x - %04umV\n",
          adc_val, adc_val, (adc_val * ADC_REF_VOLTAGE) / ADC_MAX_VALUE);
}

/////////////////////////////////////////////////////////////

static void test_set_speed(void)
{
   char input_buf[16];
   unsigned duty_value;
   unsigned direction;

   printf("Enter duty [0-%u]: ", motor_pwm_max_duty());
   fflush(stdout);

   fgets(input_buf, 16, stdin);
   sscanf(input_buf, "%u", &duty_value);

   printf("Enter dir [1=forward, 0=back]: ");
   fflush(stdout);

   fgets(input_buf, 16, stdin);
   sscanf(input_buf, "%u", &direction);

   motor_ctrl(direction, duty_value);
}

/////////////////////////////////////////////////////////////

static void test_brake(void)
{
   motor_ctrl(false, 0);
}

/////////////////////////////////////////////////////////////

static void test_zero_encoder(void)
{
   motor_zero_shaft_position();
}

/////////////////////////////////////////////////////////////

static void test_get_encoder(void)
{
   uint32_t pos = motor_get_shaft_position();
   uint32_t deg = (pos * 360) / motor_shaft_max_position();

   printf("POS : %04lu - 0x%03lx - DEG : %03lu\n",
          pos, pos, deg);
}

/////////////////////////////////////////////////////////////

static void test_dynamic_speed(void)
{
   char key;
   bool forward = true;
   uint32_t duty = 0;
   uint16_t adc_mv = 0;
   uint32_t pos_deg = 0;

   motor_ctrl(false, 0);

   // execute until 'q' key is pressed
   do
   {
      key = uart_poll();

      adc_mv = (adc_get_value() * ADC_REF_VOLTAGE) / ADC_MAX_VALUE;
      pos_deg = (motor_get_shaft_position() * 360) / motor_shaft_max_position();

      if (adc_mv < 500)
      {
         // allow change direction
         if (key == 'f')
         {
            forward = true;
         }
         else if (key == 'b')
         {
            forward = false;
         }

         duty = 0;
      }
      else if ( (adc_mv >= 500) && (adc_mv <= 2900) )
      {
         // motor speed linear control
         duty = ((adc_mv-500) * motor_pwm_max_duty()) / 2400;
      }
      else if (adc_mv > 2400)
      {
         // motor speed maximum
         duty = motor_pwm_max_duty();
      }

      // apply motor speed control
      if (duty)
      {
         motor_ctrl(forward, duty);
      }
      else
      {
         motor_ctrl(false, 0);
      }

      printf("ADC:%04umV, DUT:%04lu, POS:%03lu\n", adc_mv, duty, pos_deg);
      vTaskDelay(pdMS_TO_TICKS(50));

   } while (key != 'q') ;

   motor_ctrl(false, 0);
}

/////////////////////////////////////////////////////////////

static void test_position_control(void)
{
   char key;
   enum mpc_fsm_event event;

   mpc_fsm_init();

   char input_buf[16];
   printf("(i)nit, (c)alibrate, (z)ero, (p)osition\n");
   printf("press enter to start...\n");
   fflush(stdout);
   fgets(input_buf, 16, stdin);

   // execute until 'q' key is pressed
   do
   {
      key = uart_poll();

      switch (key)
      {
      case 'i':
         // initialize
         event = MPC_FSM_EVENT_BUT_INIT;
         break;
      case 'z':
         // zero shaft (init or calibration)
         event = MPC_FSM_EVENT_BUT_ZERO;
         break;
      case 'c':
         // calibration
         event = MPC_FSM_EVENT_BUT_CALIB;
         break;
      case 'p':
         // start position control
         event = MPC_FSM_EVENT_BUT_POS;
         break;
      default:
         event = MPC_FSM_EVENT_NONE;
      }

      mpc_fsm_execute(event);
      vTaskDelay(pdMS_TO_TICKS(100));

   } while (key != 'q') ;

   mpc_fsm_execute(MPC_FSM_STATE_INIT);
}

/////////////////////////////////////////////////////////////

static void print_test_menu(void)
{
  printf("\n");
  printf("heap-free: %u\n", xPortGetFreeHeapSize());
  printf("-----------------------------------\n");
  printf("--          TEST MENU            --\n");
  printf("-----------------------------------\n");
  printf("  2. read adc\n");
  printf(" 10. set speed\n");
  printf(" 11. brake\n");
  printf(" 20. zero encoder\n");
  printf(" 21. get encoder\n");
  printf(" 30. dynamic speed\n");
  printf(" 40. position control\n");
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
         case 2:
            test_read_adc();
            break;
         case 10:
            test_set_speed();
            break;
         case 11:
            test_brake();
            break;
         case 20:
            test_zero_encoder();
            break;
         case 21:
            test_get_encoder();
            break;
         case 30:
            test_dynamic_speed();
            break;
         case 40:
            test_position_control();
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
   adc_init();
   motor_init();

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
