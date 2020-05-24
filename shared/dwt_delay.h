#ifndef __SHARED_DWT_DELAY_H__
#define __SHARED_DWT_DELAY_H__

#include <stdint.h>

void dwt_delay_init(void);

void dwt_delay(uint32_t us);

#endif
