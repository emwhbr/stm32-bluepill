//
// To use libopencm3 with FreeRTOS on Cortex-M3 platform,
// we must connect FreeRTOS and libopencm3 so that the FreeRTOS
// functions are called for certain interrupts.
//

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>

#include <FreeRTOS.h>
#include <task.h>

// FreeRTOS exception handlers
void xPortPendSVHandler( void ) __attribute__ (( naked ));
void xPortSysTickHandler( void );
void vPortSVCHandler( void ) __attribute__ (( naked ));

// Below are the libopencm3 exception handlers required for task management and context switches.
// These handlers are invoked by the libopencm3 framework.
// By calling the FreeRTOS variant here, we keep FreeRTOS happy.

/////////////////////////////////////////////////////////////

void sv_call_handler(void)
{
   vPortSVCHandler();
}

/////////////////////////////////////////////////////////////

void pend_sv_handler(void)
{
   xPortPendSVHandler();
}

/////////////////////////////////////////////////////////////

void sys_tick_handler(void)
{
   xPortSysTickHandler();
}
