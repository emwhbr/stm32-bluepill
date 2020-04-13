#ifndef SHARED_SYSTICK_H
#define SHARED_SYSTICK_H

#include <stdint.h>

void systick_init(void);
void systick_delay_ms(uint32_t ms);
uint32_t systick_get_ms(void);

#endif
