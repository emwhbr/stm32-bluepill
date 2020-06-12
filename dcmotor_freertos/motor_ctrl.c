#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>

#include "motor_ctrl.h"

/////////////////////////////////////////////////////////////

// GPIO support
#define MOTOR_GPIO_RCC      RCC_GPIOB
#define MOTOR_GPIO_PORT     GPIOB
#define MOTOR_GPIO_PIN_DIR  GPIO3  // MAX14870 - DIR

#define MOTOR_DIR_HI  gpio_set(MOTOR_GPIO_PORT,   MOTOR_GPIO_PIN_DIR)
#define MOTOR_DIR_LO  gpio_clear(MOTOR_GPIO_PORT, MOTOR_GPIO_PIN_DIR)

// PWM (timer) support
#define MOTOR_TIMER_RCC     RCC_TIM3
#define MOTOR_TIMER_RST     RST_TIM3
#define MOTOR_TIMER         TIM3
#define MOTOR_TIMER_PWM_OC  TIM_OC1

#define MOTOR_TIMER_ALT_GPIO_RCC         RCC_GPIOB
#define MOTOR_TIMER_ALT_GPIO_PORT        GPIOB
#define MOTOR_TIMER_ALT_GPIO_PIN_PWM_OC  GPIO4

#define MOTOR_PWM_CYCLE_TIME  (1000) // [us]

// internal functions
static void motor_ctrl_gpio_init(void);
static void motor_ctrl_pwm_init(void);

/////////////////////////////////////////////////////////////

static void motor_ctrl_gpio_init(void)
{
   rcc_periph_clock_enable(MOTOR_GPIO_RCC);

   gpio_set_mode(
      MOTOR_GPIO_PORT,
      GPIO_MODE_OUTPUT_50_MHZ,
      GPIO_CNF_OUTPUT_PUSHPULL,
      MOTOR_GPIO_PIN_DIR);
}

/////////////////////////////////////////////////////////////

static void motor_ctrl_pwm_init(void)
{
   rcc_periph_clock_enable(MOTOR_TIMER_RCC);
   rcc_periph_reset_pulse(MOTOR_TIMER_RST);
   timer_disable_counter(MOTOR_TIMER);

   // remap PWM output to alternate pin
   rcc_periph_clock_enable(RCC_AFIO);
   rcc_periph_clock_enable(MOTOR_TIMER_ALT_GPIO_RCC);

   gpio_primary_remap(
      AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_OFF,
      AFIO_MAPR_TIM3_REMAP_PARTIAL_REMAP);

   gpio_set_mode(
      MOTOR_TIMER_ALT_GPIO_PORT,
      GPIO_MODE_OUTPUT_50_MHZ,
      GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
      MOTOR_TIMER_ALT_GPIO_PIN_PWM_OC);

   timer_set_mode(MOTOR_TIMER,
                  TIM_CR1_CKD_CK_INT,
                  TIM_CR1_CMS_EDGE,
                  TIM_CR1_DIR_UP);

   // we assume SYSCLK = 72MHz
   // prescaler 72 => F = 72MHz / 72= 1MHz, T = 1us
   timer_set_prescaler(MOTOR_TIMER, 72);

   // PWM frequency
   timer_enable_preload(MOTOR_TIMER);
   timer_set_counter(MOTOR_TIMER, 1);
   timer_continuous_mode(MOTOR_TIMER);
   timer_set_period(MOTOR_TIMER, MOTOR_PWM_CYCLE_TIME - 1);

   // generate an update event to cause shadow registers
   // to be preloaded before the timer is started 
   timer_generate_event(MOTOR_TIMER, TIM_EGR_UG);
   timer_clear_flag(MOTOR_TIMER, TIM_SR_UIF);
   timer_update_on_overflow(MOTOR_TIMER);

   // PWM output channel
   timer_disable_oc_output(MOTOR_TIMER, MOTOR_TIMER_PWM_OC);
   timer_set_oc_mode(MOTOR_TIMER, MOTOR_TIMER_PWM_OC, TIM_OCM_PWM1);
   timer_set_oc_value(MOTOR_TIMER, MOTOR_TIMER_PWM_OC, 0);
   timer_enable_oc_output(MOTOR_TIMER, MOTOR_TIMER_PWM_OC);

   timer_enable_counter(MOTOR_TIMER);
}

/////////////////////////////////////////////////////////////

void motor_ctrl_init(void)
{
   motor_ctrl_gpio_init();
   motor_ctrl_pwm_init();

   motor_ctrl_pwm_set_direction(true);
   motor_ctrl_brake();
}

/////////////////////////////////////////////////////////////

void motor_ctrl_pwm_set_duty(uint16_t duty_ctrl)
{
   if (duty_ctrl >= MOTOR_PWM_CYCLE_TIME)
   {
      duty_ctrl = MOTOR_PWM_CYCLE_TIME - 1;
   }

   timer_set_oc_value(MOTOR_TIMER, MOTOR_TIMER_PWM_OC, duty_ctrl);
}

/////////////////////////////////////////////////////////////

void motor_ctrl_pwm_set_direction(bool forward)
{
   if (forward)
   {
      MOTOR_DIR_HI;
   }
   else
   {
      MOTOR_DIR_LO;
   }
}

/////////////////////////////////////////////////////////////

void motor_ctrl_brake(void)
{
   motor_ctrl_pwm_set_duty(0);
}
