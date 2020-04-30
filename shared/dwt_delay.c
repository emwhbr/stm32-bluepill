#include <libopencm3/stm32/rcc.h>

#include "dwt_delay.h"

//
// Implementation notes:
// Delay using Cortex-M3 Data Watchpoint and Trace unit (DWT).
//
// Call function 'dwt_delay_init()' once to start the free running
// cycle counter. It will over flow in (SYSCLK=72MHz):
// 0xffffffff / (72 * 1000000) = 59.7 seconds.
//
// Function 'dwt_delay' can be called multiple times and shall
// handle any overflows in the cycle counter.
//
// References:
// [1] Cortex-M3 Technical Reference Manual, revision r1p1
//     http://infocenter.arm.com
//

/////////////////////////////////////////////////////////////

// registers according to ref[1]
volatile uint32_t *DWT_CONTROL  = (uint32_t *) 0xE0001000;
volatile uint32_t *DWT_CYCCNT   = (uint32_t *) 0xE0001004;
volatile uint32_t *DEMCR        = (uint32_t *) 0xE000EDFC;

// DWT Control register
#define CYCCNTENA (1 << 0)

// Debug Exception and Monitor Control Register
#define TRCENA (1 << 24)

/////////////////////////////////////////////////////////////

void dwt_delay_init(void)
{
   *DEMCR |= TRCENA;            // enable use of the trace and debug block
   *DWT_CYCCNT = 0;             // reset cycle counter
   *DWT_CONTROL |= CYCCNTENA;   // start cycle counter
}

/////////////////////////////////////////////////////////////

void dwt_delay(uint32_t us)
{
   const uint32_t tick_begin = *DWT_CYCCNT;
   const uint32_t tick_delay = us * (rcc_ahb_frequency / 1000000);

   // check for overflow happens automatically
   while ( (*DWT_CYCCNT - tick_begin) < tick_delay ) ;
}
