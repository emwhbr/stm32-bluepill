#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include "libopencm3/cm3/nvic.h"
#include <libopencm3/stm32/exti.h>

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "ir_nec.h"

//
// Implementation notes:
// This driver implements an IR decoder for the NEC Extended Prototcol.
// Using the output from chip TSOP38238 (Vishay) to decode the IR signal.
// It's assumed that FreeRTOS is setup and configured properly.
//
// References:
// [1] IR Receiver Modules for Remote Control Systems, Document Number: 81733
//     https://www.vishay.com
//

/////////////////////////////////////////////////////////////

// GPIO support
#define TSOP38238_GPIO_RCC       RCC_GPIOB
#define TSOP38238_GPIO_PORT      GPIOB
#define TSOP38238_GPIO_PIN_INT   GPIO6  // Interrupt signal, TSOP38238:OUT (pin 1)
#define TSOP38238_GPIO_PIN_DBG   GPIO7  // Debug signal

#define TSOP38238_GPIO_PIN_INT_EXTI      EXTI6
#define TSOP38238_GPIO_PIN_INT_IRQ       NVIC_EXTI9_5_IRQ
#define TSOP38238_GPIO_PIN_INT_IRQ_PRIO  configMAX_SYSCALL_INTERRUPT_PRIORITY

#define TSOP38238_OUT  gpio_get(TSOP38238_GPIO_PORT, TSOP38238_GPIO_PIN_INT)

#define TSOP38238_DBG_HI  gpio_set(TSOP38238_GPIO_PORT,   TSOP38238_GPIO_PIN_DBG)
#define TSOP38238_DBG_LO  gpio_clear(TSOP38238_GPIO_PORT, TSOP38238_GPIO_PIN_DBG)
#define TSOP38238_DBG_TG  gpio_toggle(TSOP38238_GPIO_PORT, TSOP38238_GPIO_PIN_DBG)

// Timer support
#define TSOP38238_TIMER_RCC  RCC_TIM4
#define TSOP38238_TIMER_RST  RST_TIM4
#define TSOP38238_TIMER      TIM4

#define TSOP38238_TIMER_IRQ       NVIC_TIM4_IRQ
#define TSOP38238_TIMER_IRQ_PRIO  configMAX_SYSCALL_INTERRUPT_PRIORITY

// Timing limits for the TSOP38238 out signal.
// The values were found after empirical tests using
// different directions and distances with a remote control.
//
// Note! The prescaler defines a timer tick to 2us.

// AGC leading pulse burst, 9ms
#define T_AGC_MIN  (4300)  // 8.6ms
#define T_AGC_MAX  (4700)  // 9.4ms

// Message space,4.5ms
#define T_SPACE_LONG_MIN   (2100)  // 4.2ms
#define T_SPACE_LONG_MAX   (2400)  // 4.8ms

// Reapeat space, 2.25ms
#define T_SPACE_SHORT_MIN  (750)   // 1.5ms
#define T_SPACE_SHORT_MAX  (1500)  // 3.0ms

// Basic pule burst and logical "0" space, 562.5us
#define T_PULSE_MIN  (220) // 440us
#define T_PULSE_MAX  (350) // 700us

// Logical "1" space, 1.6875ms
#define T_L1_MIN  (750) // 1.5ms
#define T_L1_MAX  (900) // 1.8ms

// NEC Extended protocol states
typedef enum ir_nec_state {
   IR_NEC_STATE_UNKNOWN,
   IR_NEC_STATE_AGC,
   IR_NEC_STATE_SPACE,
   IR_NEC_STATE_DATA,
   IR_NEC_STATE_END_DATA,
   IR_NEC_STATE_REPEAT,
} IR_NEC_STATE;

// internal driver data (state and synchronization)
struct ir_nec_dev
{
   IR_NEC_STATE  state;
   uint8_t       timeout;
   uint32_t      t_error; // for debug and tuning purposes
   uint32_t      error;   // for debug and tuning purposes
   uint32_t      t_last;
   uint8_t       data_bit;
   uint32_t      data_word;
   TaskHandle_t  xEXTIWorkTask;
   QueueHandle_t xEXTIQueue;
};

static struct ir_nec_dev *g_dev = NULL;

// internal functions
static void ir_nec_gpio_init(void);
static void ir_nec_timer_init(void);
static void ir_nec_enable_ext_int(void);
static void ir_nec_int_work_task(__attribute__((unused))void * pvParameters);

/////////////////////////////////////////////////////////////

static void ir_nec_gpio_init(void)
{
   rcc_periph_clock_enable(TSOP38238_GPIO_RCC);

   gpio_set_mode(
      TSOP38238_GPIO_PORT,
      GPIO_MODE_INPUT,
      GPIO_CNF_INPUT_FLOAT,
      TSOP38238_GPIO_PIN_INT);

   gpio_set_mode(
      TSOP38238_GPIO_PORT,
      GPIO_MODE_OUTPUT_50_MHZ,
      GPIO_CNF_OUTPUT_PUSHPULL,
      TSOP38238_GPIO_PIN_DBG);

   TSOP38238_DBG_HI;
}

/////////////////////////////////////////////////////////////

static void ir_nec_timer_init(void)
{
   rcc_periph_clock_enable(TSOP38238_TIMER_RCC);
   rcc_periph_reset_pulse(TSOP38238_TIMER_RST);

   timer_disable_counter(TSOP38238_TIMER);

   timer_set_mode(TSOP38238_TIMER,
                  TIM_CR1_CKD_CK_INT,
                  TIM_CR1_CMS_EDGE,
                  TIM_CR1_DIR_UP);

   // we assume SYSCLK = 72MHz
   // prescaler 144 => F = 72MHz / 144 = 0.5MHz, T = 2us
   timer_set_prescaler(TSOP38238_TIMER, 144);

   // We want a 85ms timeout for the TSOP38238 out signal.
   // Period time between messages is approx. 108ms.
   timer_enable_preload(TSOP38238_TIMER);
   timer_set_counter(TSOP38238_TIMER, 1);
   timer_continuous_mode(TSOP38238_TIMER);
   timer_set_period(TSOP38238_TIMER, 42500);  // 42500 * 2us => 85ms
   //timer_set_period(TSOP38238_TIMER, 20000);

   // generate an update event to cause shadow registers
   // to be preloaded before the timer is started 
   timer_generate_event(TSOP38238_TIMER, TIM_EGR_UG);
   timer_clear_flag(TSOP38238_TIMER, TIM_SR_UIF);
   timer_update_on_overflow(TSOP38238_TIMER);

   timer_enable_irq(TSOP38238_TIMER, TIM_DIER_UIE);
   nvic_set_priority(TSOP38238_TIMER_IRQ, TSOP38238_TIMER_IRQ_PRIO);
   nvic_enable_irq(TSOP38238_TIMER_IRQ);
}

/////////////////////////////////////////////////////////////

// This is the ISR for the timer used to measure
// elpased time for the TSOP38238 out signal.

void tim4_isr(void)
{
   if (timer_get_flag(TSOP38238_TIMER, TIM_SR_UIF))
   {
      timer_disable_counter(TSOP38238_TIMER);
      timer_clear_flag(TSOP38238_TIMER, TIM_SR_UIF);
      g_dev->timeout = 1;

      // signal worker task (we are using the EXTI worker because it's doing all the work)
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      vTaskNotifyGiveFromISR(g_dev->xEXTIWorkTask, &xHigherPriorityTaskWoken);
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
   }
}

/////////////////////////////////////////////////////////////

static void ir_nec_enable_ext_int(void)
{
   // configure external interrupt
   rcc_periph_clock_enable(RCC_AFIO);

   exti_select_source(TSOP38238_GPIO_PIN_INT_EXTI, TSOP38238_GPIO_PORT);
   exti_set_trigger(TSOP38238_GPIO_PIN_INT_EXTI, EXTI_TRIGGER_BOTH);
   exti_enable_request(TSOP38238_GPIO_PIN_INT_EXTI);

   nvic_set_priority(TSOP38238_GPIO_PIN_INT_IRQ, TSOP38238_GPIO_PIN_INT_IRQ_PRIO);
   nvic_enable_irq(TSOP38238_GPIO_PIN_INT_IRQ);
}

/////////////////////////////////////////////////////////////

// This is the ISR for external interrupts generated by TSOP38238

void exti9_5_isr(void)
{
   if (exti_get_flag_status(TSOP38238_GPIO_PIN_INT_EXTI) == TSOP38238_GPIO_PIN_INT_EXTI)
   {
      // disable external interrupts from chip
      exti_disable_request(TSOP38238_GPIO_PIN_INT_EXTI);
      exti_reset_request(TSOP38238_GPIO_PIN_INT_EXTI);

      // signal worker task
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      vTaskNotifyGiveFromISR(g_dev->xEXTIWorkTask, &xHigherPriorityTaskWoken);
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
   }
}

/////////////////////////////////////////////////////////////

static void ir_nec_int_work_task(__attribute__((unused))void * pvParameters)
{
   while (1)
   {
      // wait for signal from ISR (EXTI or timer)
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

      uint8_t out = TSOP38238_OUT;
      IR_NEC_STATE next_state = g_dev->state;
      uint8_t timing_ok = 1;
      uint32_t t_delta;
      uint32_t t_now;

      // check if timeout detected in timer ISR
      if (g_dev->timeout)
      {
         TSOP38238_DBG_TG;
         g_dev->state = IR_NEC_STATE_UNKNOWN;
         g_dev->timeout = 0;
         timing_ok = 0;
      }

      switch (g_dev->state)
      {
         case IR_NEC_STATE_UNKNOWN:
            if (!out && timing_ok)
            {
               // falling edge, assume start of AGC
               TSOP38238_DBG_TG;
               timer_set_counter(TSOP38238_TIMER, 1);
               timer_enable_counter(TSOP38238_TIMER);
               g_dev->timeout = 0;
               next_state = IR_NEC_STATE_AGC;
            }
            break;
         case IR_NEC_STATE_AGC:
            if (out)
            {
               // rising edge, assume end of AGC
               g_dev->t_last = timer_get_counter(TSOP38238_TIMER);

               // check AGC pulse length
               if ( (g_dev->t_last > T_AGC_MIN) && (g_dev->t_last < T_AGC_MAX) )
               {
                  // timing ok, assume start of SPACE
                  next_state = IR_NEC_STATE_SPACE;
               }
               else
               {
                  g_dev->t_error = g_dev->t_last;
                  timing_ok = 0;
               }
            }
            break;
         case IR_NEC_STATE_SPACE:
            if (!out)
            {
               // falling edge, assume end of SPACE
               t_now = timer_get_counter(TSOP38238_TIMER);
               t_delta = t_now - g_dev->t_last;

               // check SPACE pulse length
               if ( (t_delta > T_SPACE_LONG_MIN) && (t_delta < T_SPACE_LONG_MAX) )
               {
                  // timing matches long SPACE, assume start of DATA
                  g_dev->t_last = t_now;
                  g_dev->data_bit = 0;
                  g_dev->data_word = 0;
                  next_state = IR_NEC_STATE_DATA;
               }
               else if ( (t_delta > T_SPACE_SHORT_MIN) && (t_delta < T_SPACE_SHORT_MAX) )
               {
                  // timing matches short SPACE, assume start of REPEAT
                  g_dev->t_last = t_now;
                  next_state = IR_NEC_STATE_REPEAT;
               }
               else
               {
                  g_dev->t_error = t_delta;
                  timing_ok = 0;
               }
            }
            break;
         case IR_NEC_STATE_DATA:
            t_now = timer_get_counter(TSOP38238_TIMER);
            t_delta = t_now - g_dev->t_last;
            g_dev->t_last = t_now;
            if (out)
            {
               // rising edge, start of a data bit (logical "0" or "1")
               if ( (t_delta < T_PULSE_MIN) || (t_delta > T_PULSE_MAX) )
               {
                  g_dev->t_error = t_delta;
                  timing_ok = 0;
               }
            }
            else
            {
               // falling edge, end of a data bit, check if logical "0" or "1"
               if ( (t_delta > T_L1_MIN) && (t_delta < T_L1_MAX) )
               {
                  // logical "1"
                  g_dev->data_word |= (1 << g_dev->data_bit);
               }
               else if ( (t_delta > T_PULSE_MIN) && (t_delta < T_PULSE_MAX) )
               {
                  // logical "0"
                  g_dev->data_word &= ~(1 << g_dev->data_bit); 
               }
               else
               {
                  g_dev->t_error = t_delta;
                  timing_ok = 0;
               }

               // check if all data bits have been received
               if ( timing_ok && (++g_dev->data_bit == 32) ) 
               {
                  next_state = IR_NEC_STATE_END_DATA;
               }
            }
            break;
         case IR_NEC_STATE_END_DATA:
            if (out)
            {
               // rising edge
               t_now = timer_get_counter(TSOP38238_TIMER);
               t_delta = t_now - g_dev->t_last;

               // check final pulse length
               if ( (t_delta > T_PULSE_MIN) && (t_delta < T_PULSE_MAX) )
               {
                  timer_disable_counter(TSOP38238_TIMER);
                  next_state = IR_NEC_STATE_UNKNOWN;

                  // SIGNAL DATA-32bit
                  xQueueSend(g_dev->xEXTIQueue, (const void *) &g_dev->data_word, 0);
               }
               else
               {
                  g_dev->t_error = t_delta;
                  timing_ok = 0;
               }
            }
            break;
         case IR_NEC_STATE_REPEAT:
            if (out)
            {
               // rising edge
               t_now = timer_get_counter(TSOP38238_TIMER);
               t_delta = t_now - g_dev->t_last;

               // check final pulse length
               if ( (t_delta > T_PULSE_MIN) && (t_delta < T_PULSE_MAX) )
               {
                  timer_disable_counter(TSOP38238_TIMER);
                  next_state = IR_NEC_STATE_UNKNOWN;

                  // SIGNAL REPEAT CODE
                  g_dev->data_word = IR_NEC_MSG_REPEAT_CODE;
                  xQueueSend(g_dev->xEXTIQueue, (const void *) &g_dev->data_word, 0);
               }
               else
               {
                  g_dev->t_error = t_delta;
                  timing_ok = 0;
               }
            }
            break;
      }

      // check if signal timing is within limits
      if (!timing_ok)
      {
         timer_disable_counter(TSOP38238_TIMER);
         next_state = IR_NEC_STATE_UNKNOWN;
      }

      // check if transition to new state
      if (next_state != g_dev->state)
      {
         g_dev->state = next_state;
      }

      // for debug and tuning purposes (handy breakpoint)
      if (!timing_ok)
      {
         __asm("nop");
         g_dev->error++;
      }

      // re-enable interrupts from chip
      exti_enable_request(TSOP38238_GPIO_PIN_INT_EXTI);
   }
}

/////////////////////////////////////////////////////////////

void ir_nec_init(UBaseType_t int_work_task_prio)
{
   ir_nec_gpio_init();
   ir_nec_timer_init();

   if (g_dev == NULL)
   {
      g_dev = pvPortMalloc(sizeof(struct ir_nec_dev));
   }
   g_dev->state = IR_NEC_STATE_UNKNOWN;
   g_dev->timeout = 0;
   g_dev->error = 0;
   g_dev->data_bit = 0;
   g_dev->data_word = 0;

   if (g_dev->xEXTIWorkTask == NULL)
   {
      // Worker task for external interrupts generated by TSOP38238.
      // Minium stack size is TBD words (checked with configCHECK_FOR_STACK_OVERFLOW).
      xTaskCreate(ir_nec_int_work_task, "IRINTWRK", 250, NULL, int_work_task_prio, &g_dev->xEXTIWorkTask);
   }

   if (g_dev->xEXTIQueue == NULL)
   {
      // Queue used by worker task to produce messages,
      // Size is 5 elements == 0.5s when time between messages is approx. 108ms.
      g_dev->xEXTIQueue = xQueueCreate(5, sizeof(IR_NEC_MSG));
   }

   // enable external interrupts
   ir_nec_enable_ext_int();
}

/////////////////////////////////////////////////////////////

nec_err_t ir_nec_recv(IR_NEC_MSG *msg,
                      TickType_t xTicksToWait)
{
   if (g_dev->xEXTIQueue == NULL)
   {
      return NEC_FAIL;
   }

   if (xQueueReceive(g_dev->xEXTIQueue,
                     msg,
                     xTicksToWait) != pdPASS)
   {
      return NEC_TIMEOUT;
   }

   return NEC_OK;
}
