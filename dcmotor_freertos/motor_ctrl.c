#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>

#include "motor_ctrl.h"

//
// Implementation notes:
// This driver controls the brushed DC motor driver DRV8871
// from Texas Instrumens.
//
// References:
// [1] DRV8871 Datasheet, Document Number: SLVSCY9B
//     https://www.ti.com
//

/////////////////////////////////////////////////////////////

// PWM (timer) support
#define MOTOR_TIMER_RCC     RCC_TIM3
#define MOTOR_TIMER_RST     RST_TIM3
#define MOTOR_TIMER         TIM3
#define MOTOR_TIMER_OC_IN1  TIM_OC1 // DRV8871 - Pin IN1
#define MOTOR_TIMER_OC_IN2  TIM_OC2 // DRV8871 - Pin IN2

#define MOTOR_TIMER_ALT_GPIO_RCC         RCC_GPIOB
#define MOTOR_TIMER_ALT_GPIO_PORT        GPIOB
#define MOTOR_TIMER_ALT_GPIO_PIN_OC_IN1  GPIO4
#define MOTOR_TIMER_ALT_GPIO_PIN_OC_IN2  GPIO5

#define MOTOR_PWM_CYCLE_TIME  MOTOR_PWM_MAX_DUTY // [ x 0.125us]

// internal functions
static void motor_ctrl_pwm_init(void);

/////////////////////////////////////////////////////////////

static void motor_ctrl_pwm_init(void)
{
   rcc_periph_clock_enable(MOTOR_TIMER_RCC);
   rcc_periph_reset_pulse(MOTOR_TIMER_RST);
   timer_disable_counter(MOTOR_TIMER);

   // remap PWM outputs to alternate pins
   rcc_periph_clock_enable(RCC_AFIO);
   rcc_periph_clock_enable(MOTOR_TIMER_ALT_GPIO_RCC);

   gpio_primary_remap(
      AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_OFF,
      AFIO_MAPR_TIM3_REMAP_PARTIAL_REMAP);

   gpio_set_mode(
      MOTOR_TIMER_ALT_GPIO_PORT,
      GPIO_MODE_OUTPUT_50_MHZ,
      GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
      MOTOR_TIMER_ALT_GPIO_PIN_OC_IN1 | MOTOR_TIMER_ALT_GPIO_PIN_OC_IN2);

   timer_set_mode(MOTOR_TIMER,
                  TIM_CR1_CKD_CK_INT,
                  TIM_CR1_CMS_EDGE,
                  TIM_CR1_DIR_UP);

   // we assume SYSCLK = 72MHz
   // prescaler 9 => F = 72MHz / 9 = 8MHz, T = 0.125us
   // timer period = 0 - 65535 => 0 - 8191us
   timer_set_prescaler(MOTOR_TIMER, 8); // 9 = 8 + 1

   // PWM frequency
   timer_enable_preload(MOTOR_TIMER);
   timer_set_counter(MOTOR_TIMER, 0);
   timer_continuous_mode(MOTOR_TIMER);
   timer_set_period(MOTOR_TIMER, MOTOR_PWM_CYCLE_TIME);

   // generate an update event to cause shadow registers
   // to be preloaded before the timer is started 
   timer_generate_event(MOTOR_TIMER, TIM_EGR_UG);
   timer_clear_flag(MOTOR_TIMER, TIM_SR_UIF);
   timer_update_on_overflow(MOTOR_TIMER);

   // PWM output channels
   timer_disable_oc_output(MOTOR_TIMER, MOTOR_TIMER_OC_IN1);
   timer_disable_oc_output(MOTOR_TIMER, MOTOR_TIMER_OC_IN2);

   // outputs are active low
   timer_set_oc_polarity_low(MOTOR_TIMER, MOTOR_TIMER_OC_IN1);
   timer_set_oc_polarity_low(MOTOR_TIMER, MOTOR_TIMER_OC_IN2);

   // make outputs inactive (high)
   timer_set_oc_mode(MOTOR_TIMER, MOTOR_TIMER_OC_IN1, TIM_OCM_FORCE_LOW);
   timer_set_oc_mode(MOTOR_TIMER, MOTOR_TIMER_OC_IN2, TIM_OCM_FORCE_LOW);

   timer_enable_oc_output(MOTOR_TIMER, MOTOR_TIMER_OC_IN1);
   timer_enable_oc_output(MOTOR_TIMER, MOTOR_TIMER_OC_IN2);
}

/////////////////////////////////////////////////////////////

void motor_ctrl_init(void)
{
   motor_ctrl_pwm_init();

   motor_ctrl_brake();
}

/////////////////////////////////////////////////////////////

void motor_ctrl_set_speed(bool forward, uint16_t duty_ctrl)
{
   if (duty_ctrl >= MOTOR_PWM_CYCLE_TIME)
   {
      duty_ctrl = MOTOR_PWM_CYCLE_TIME;
   }

   if (forward)
   {
      // IN1 = 1 (inactive), IN2 = PWM
      timer_set_oc_mode(MOTOR_TIMER, MOTOR_TIMER_OC_IN1, TIM_OCM_FORCE_LOW);
      timer_set_oc_mode(MOTOR_TIMER, MOTOR_TIMER_OC_IN2, TIM_OCM_PWM1);
      timer_set_oc_value(MOTOR_TIMER, MOTOR_TIMER_OC_IN2, duty_ctrl);
   }
   else
   {
      // IN1 = PWM, IN2 = 1 (inactive)
      timer_set_oc_mode(MOTOR_TIMER, MOTOR_TIMER_OC_IN1, TIM_OCM_PWM1);
      timer_set_oc_value(MOTOR_TIMER, MOTOR_TIMER_OC_IN1, duty_ctrl);
      timer_set_oc_mode(MOTOR_TIMER, MOTOR_TIMER_OC_IN2, TIM_OCM_FORCE_LOW);
   }

   timer_enable_counter(MOTOR_TIMER);
}

/////////////////////////////////////////////////////////////

void motor_ctrl_brake(void)
{
   timer_set_oc_mode(MOTOR_TIMER, MOTOR_TIMER_OC_IN1, TIM_OCM_FORCE_LOW);
   timer_set_oc_mode(MOTOR_TIMER, MOTOR_TIMER_OC_IN2, TIM_OCM_FORCE_LOW);
   timer_disable_counter(MOTOR_TIMER);
}
