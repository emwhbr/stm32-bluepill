#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>

#include "adc.h"

/////////////////////////////////////////////////////////////

// GPIO support
#define ADC_GPIO_RCC      RCC_GPIOA
#define ADC_GPIO_PORT     GPIOA
#define ADC_GPIO_PIN_IN0  GPIO2

// ADC peripheral
#define ADC_RCC          RCC_ADC1
#define ADC_RST          RST_ADC1
#define ADC_UNIT         ADC1
#define ADC_CHANNEL_IN0  ADC_CHANNEL2

// internal functions
static void adc_gpio_init(void);
static void adc_peripherial_init(void);

/////////////////////////////////////////////////////////////

static void adc_gpio_init(void)
{
   rcc_periph_clock_enable(ADC_GPIO_RCC);

   gpio_set_mode(
      ADC_GPIO_PORT,
      GPIO_MODE_INPUT,
      GPIO_CNF_INPUT_ANALOG,
      ADC_GPIO_PIN_IN0);
}

/////////////////////////////////////////////////////////////

static void adc_peripherial_init(void)
{
   // We assume SYSCLK = 72MHz. ADC input clock is derived from
   // PCLK2 (SYSCLK / 1). According to reference manual,
   // ADC input clock must not exceed 14MHz. Setting prescaler to 6,
   // results in 12MHz ( (72 / 1) / 6 ).
   rcc_set_adcpre(RCC_CFGR_ADCPRE_PCLK2_DIV6);

   rcc_periph_clock_enable(ADC_RCC);
   rcc_periph_reset_pulse(ADC_RST);

   adc_power_off(ADC_UNIT);

   adc_set_dual_mode(ADC_CR1_DUALMOD_IND);
   adc_disable_scan_mode(ADC_UNIT);
   adc_set_right_aligned(ADC_UNIT);
   adc_set_single_conversion_mode(ADC_UNIT);
   adc_set_sample_time(ADC_UNIT, ADC_CHANNEL_IN0, ADC_SMPR_SMP_239DOT5CYC);
   adc_enable_external_trigger_regular(ADC_UNIT, ADC_CR2_EXTSEL_SWSTART);

   adc_power_on(ADC_UNIT);

   adc_reset_calibration(ADC_UNIT);
   adc_calibrate(ADC_UNIT);
}

/////////////////////////////////////////////////////////////

void adc_init(void)
{
   adc_gpio_init();
   adc_peripherial_init();
}

/////////////////////////////////////////////////////////////

uint16_t adc_get_value(void)
{
   uint8_t channel = ADC_CHANNEL_IN0;
   adc_set_regular_sequence(ADC_UNIT, 1, &channel);

   adc_start_conversion_regular(ADC_UNIT);
   while (!adc_eoc(ADC_UNIT));

   return adc_read_regular(ADC_UNIT);
}
