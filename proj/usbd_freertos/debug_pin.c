#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include <stdint.h>

#include "debug_pin.h"

// GPIO support
#define DEBUG_GPIO_RCC       RCC_GPIOB
#define DEBUG_GPIO_PORT      GPIOB
#define DEBUG_GPIO_PIN_DBG1  GPIO9  // Debug signal
#define DEBUG_GPIO_PIN_DBG2  GPIO8  // Debug signal
#define DEBUG_GPIO_PIN_DBG3  GPIO7  // Debug signal
#define DEBUG_GPIO_PIN_DBG4  GPIO6  // Debug signal

/////////////////////////////////////////////////////////////

static void debug_pin_gpio_init(void)
{
   rcc_periph_clock_enable(DEBUG_GPIO_RCC);

   gpio_set_mode(
      DEBUG_GPIO_PORT,
      GPIO_MODE_OUTPUT_50_MHZ,
      GPIO_CNF_OUTPUT_PUSHPULL,
      DEBUG_GPIO_PIN_DBG1 | DEBUG_GPIO_PIN_DBG2 |
      DEBUG_GPIO_PIN_DBG3 | DEBUG_GPIO_PIN_DBG4);
}

/////////////////////////////////////////////////////////////

void debug_pin_init(void)
{
   debug_pin_gpio_init();

   DBG_P1_LO;
   DBG_P2_LO;
   DBG_P3_LO;
   DBG_P4_LO;
}

/////////////////////////////////////////////////////////////

void debug_pin_set_value(debug_pin_t pin, bool value)
{
   uint16_t gpio = 0;

   switch(pin)
   {
      case DEBUG_PIN1:
         gpio = DEBUG_GPIO_PIN_DBG1;
         break;
      case DEBUG_PIN2:
         gpio = DEBUG_GPIO_PIN_DBG2;
         break;
      case DEBUG_PIN3:
         gpio = DEBUG_GPIO_PIN_DBG3;
         break;
      case DEBUG_PIN4:
         gpio = DEBUG_GPIO_PIN_DBG4;
         break;
      default:
         return;
   }

   if (value)
   {
      gpio_set(DEBUG_GPIO_PORT, gpio);
   }
   else
   {
      gpio_clear(DEBUG_GPIO_PORT, gpio);
   }
}

