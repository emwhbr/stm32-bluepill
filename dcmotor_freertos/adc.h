#ifndef __ADC_H__
#define __ADC_H__

#include <stdint.h>

#define ADC_REF_VOLTAGE  3300  // [mV]
#define ADC_MAX_VALUE    4095  // 12-bit ADC (0x000 - 0xfff)

void adc_init(void);

uint16_t adc_get_value(void);

#endif
