#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>

#include "motor_encoder.h"

//
// Implementation notes:
// This driver decodes the signals from the quadrature encoder
// used by Pololu DC motor product #3240
//
// References:
// [1] Pololu #3240 spec
//     https://www.pololu.com/product/3240
//

/////////////////////////////////////////////////////////////

// GPIO support
#define ENCODER_GPIO_RCC       RCC_GPIOA
#define ENCODER_GPIO_PORT      GPIOA
#define ENCODER_GPIO_PIN_ENCA  GPIO0  // Encoder signal A
#define ENCODER_GPIO_PIN_ENCB  GPIO1  // Encoder signal B

// Timer support
#define ENCODER_TIMER_RCC  RCC_TIM2
#define ENCODER_TIMER_RST  RST_TIM2
#define ENCODER_TIMER      TIM2

#define ENCODER_TIMER_ENCA_IC     TIM_IC1
#define ENCODER_TIMER_ENCA_IC_IN  TIM_IC_IN_TI1
#define ENCODER_TIMER_ENCB_IC     TIM_IC2
#define ENCODER_TIMER_ENCB_IC_IN  TIM_IC_IN_TI2

// internal functions
static void motor_encoder_gpio_init(void);
static void motor_encoder_timer_init(void);

/////////////////////////////////////////////////////////////

static void motor_encoder_gpio_init(void)
{
   rcc_periph_clock_enable(ENCODER_GPIO_RCC);

   gpio_set_mode(
      ENCODER_GPIO_PORT,
      GPIO_MODE_INPUT,
      GPIO_CNF_INPUT_FLOAT,
      ENCODER_GPIO_PIN_ENCA | ENCODER_GPIO_PIN_ENCB);
}

/////////////////////////////////////////////////////////////

static void motor_encoder_timer_init(void)
{
   rcc_periph_clock_enable(ENCODER_TIMER_RCC);
   rcc_periph_reset_pulse(ENCODER_TIMER_RST);

   timer_disable_counter(ENCODER_TIMER);

   // encoder mode, count edges on both input channels
   timer_slave_set_mode(ENCODER_TIMER, TIM_SMCR_SMS_EM3);
   timer_set_period(ENCODER_TIMER, MOTOR_ENCODER_CPR_GEAR_SHAFT);

   timer_ic_set_input(ENCODER_TIMER, ENCODER_TIMER_ENCA_IC, ENCODER_TIMER_ENCA_IC_IN);
   timer_ic_set_input(ENCODER_TIMER, ENCODER_TIMER_ENCB_IC, ENCODER_TIMER_ENCB_IC_IN);

   // encoder signals are pulled low when activated
   timer_ic_set_polarity(ENCODER_TIMER, ENCODER_TIMER_ENCA_IC, TIM_IC_FALLING);
   timer_ic_set_polarity(ENCODER_TIMER, ENCODER_TIMER_ENCB_IC, TIM_IC_FALLING);

   timer_ic_enable(ENCODER_TIMER, ENCODER_TIMER_ENCA_IC);
   timer_ic_enable(ENCODER_TIMER, ENCODER_TIMER_ENCB_IC);
   timer_enable_counter(ENCODER_TIMER);
}

/////////////////////////////////////////////////////////////

void motor_encoder_init(void)
{
   motor_encoder_gpio_init();
   motor_encoder_timer_init();

   motor_encoder_zero_gearbox_shaft_pos();
}

/////////////////////////////////////////////////////////////

void motor_encoder_zero_gearbox_shaft_pos(void)
{
   timer_set_counter(ENCODER_TIMER, 0);
}

/////////////////////////////////////////////////////////////

uint32_t motor_encoder_get_gearbox_shaft_pos(void)
{
   return timer_get_counter(ENCODER_TIMER);
}
