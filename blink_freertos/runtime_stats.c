#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>

//
// FreeRTOS is configured for the collection of runtime statistics.
// We need to configure a timer/counter and the counter should be
// at least 10 times the frequency of the tick count.
//
// Provide definitions for
// - portCONFIGURE_TIMER_FOR_RUN_TIME_STATS
// - portGET_RUN_TIME_COUNTER_VALUE
//
// We are using Timer 2 as the timer/counter.
//

void vConfigureTimerForRunTimeStats(void);
uint32_t ulGetRunTimeCounterValue(void);

volatile uint32_t g_runtime_counter = 0;

////////////////////////////////////////////////////////////

void vConfigureTimerForRunTimeStats(void)
{
   rcc_periph_clock_enable(RCC_TIM2);

   timer_disable_counter(TIM2);
   rcc_periph_reset_pulse(RST_TIM2);

   timer_set_mode(TIM2,
      TIM_CR1_CKD_CK_INT,
      TIM_CR1_CMS_EDGE,
      TIM_CR1_DIR_UP);

   timer_enable_preload(TIM2);
   timer_continuous_mode(TIM2);

   // Assume SYSCLK=72MHz, timer counts each 1us
   const uint32_t prescaler = 72;
   timer_set_prescaler(TIM2, prescaler-1);

   // APB1 bus frequency is 36 MHz (APB1 prescaler is 2).
   // When APB1 prescaler is not 1, the timer is driven by frequency x 2.
   // The timer frequency shall be at least 10 times the frequency of the FreeRTOS tick count.
   // frequency = configTICK_RATE_HZ * 10
   const uint32_t frequency = 1000 * 10;
   timer_set_period(TIM2, ((rcc_apb1_frequency * 2 / prescaler) / frequency) );

   // enable interrupt and start timer
   nvic_enable_irq(NVIC_TIM2_IRQ);
   nvic_set_priority(NVIC_TIM2_IRQ, 1);
   timer_enable_irq(TIM2, TIM_DIER_UIE);
   timer_enable_counter(TIM2);
}

/////////////////////////////////////////////////////////////

uint32_t ulGetRunTimeCounterValue(void)
{
   return g_runtime_counter;
}

/////////////////////////////////////////////////////////////

void tim2_isr(void)
{
   const uint32_t sr = TIM_SR(TIM2);
   if (sr & TIM_SR_UIF)
   {
      timer_clear_flag(TIM2, TIM_SR_UIF); // Clear interrrupt flag
      g_runtime_counter++;
   }
}
