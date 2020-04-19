# FreeRTOS

## General
This directory includes the common FreeRTOS source files and specifics from port GCC/ARM_CM3.<br/>

The files were copied from the git submodule FreeRTOS-Kernel (commit: 1abca83c891bbb0233e0d2f61a2970e377d4d534).<br/>

## FreeRTOSConfig.h
The FreeRTOSConfig.h is specific for each individual project.<br/>

Template for this file was taken from FreeRTOS v10.3.1, Container: FreeRTOSv10.3.1.zip<br/>
...FreeRTOS/Demo/CORTEX_STM32F103_Primer_GCC/FreeRTOSConfig.h<br/>

We don't use stm32f10x_lib.h because libopencm3 provides our device driver facilities.<br/>
So we comment that line out of FreeRTOSConfig.h

## opencm3_freertos.c
This was inspired by the book __Beginning STM32 : Developing with FreeRTOS, libopencm3 and GCC__ (Warren W. Gay).<br/>
Source code for the book is found at : https://github.com/ve3wwg/stm32f103c8t6.

In order to use libopencm3 with FreeRTOS on Cortex-M3 platform, some functions needs to be defined in our project.<br/>
We must connect FreeRTOS and libopencm3 so that the FreeRTOS functions are called for certain interrupts.<br/>

Detailed information can be found in the source code for libopencm3:
...libopencm3/lib/cm3/vector.c

```c
__attribute__ ((section(".vectors")))
vector_table_t vector_table = {
   .initial_sp_value = &_stack,
   .reset = reset_handler,
   .nmi = nmi_handler,
   .hard_fault = hard_fault_handler,

/* Those are defined only on CM3 or CM4 */
#if defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__)
   .memory_manage_fault = mem_manage_handler,
   .bus_fault = bus_fault_handler,
   .usage_fault = usage_fault_handler,
   .debug_monitor = debug_monitor_handler,
#endif

   .sv_call = sv_call_handler,   <------- Note! Call FreeRTOS variant in this handler.
   .pend_sv = pend_sv_handler,   <------- Note! Call FreeRTOS variant in this handler.
   .systick = sys_tick_handler,  <------- Note! Call FreeRTOS variant in this handler.
   .irq = {
      IRQ_HANDLERS
   }
};
```

## Structure
FreeRTOS<br/>
├── croutine.c<br/>
├── event_groups.c<br/>
├── heap_1.c<br/>
├── heap_2.c<br/>
├── heap_3.c<br/>
├── heap_4.c<br/>
├── heap_5.c<br/>
├── include<br/>
│   ├── atomic.h<br/>
│   ├── croutine.h<br/>
│   ├── deprecated_definitions.h<br/>
│   ├── event_groups.h<br/>
│   ├── FreeRTOS.h<br/>
│   ├── list.h<br/>
│   ├── message_buffer.h<br/>
│   ├── mpu_prototypes.h<br/>
│   ├── mpu_wrappers.h<br/>
│   ├── portable.h<br/>
│   ├── portmacro.h<br/>
│   ├── projdefs.h<br/>
│   ├── queue.h<br/>
│   ├── semphr.h<br/>
│   ├── stack_macros.h<br/>
│   ├── StackMacros.h<br/>
│   ├── stream_buffer.h<br/>
│   ├── task.h<br/>
│   └── timers.h<br/>
├── LICENSE.md<br/>
├── list.c<br/>
├── __opencm3_freertos.c <-- This is the "glue" bewteen libopencm3 and FreeRTOS__<br/>
├── port.c<br/>
├── queue.c<br/>
├── README.md.c<br/>
├── stream_buffer.c<br/>
├── tasks.c<br/>
└── timers.c<br/>
