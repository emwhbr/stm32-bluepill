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
In order to use libopencm3 with FreeRTOS on Cortex-M3 platform, some functions needs to be defined.

This was inspired by the book __Beginning STM32 : Developing with FreeRTOS, libopencm3 and GCC__ (Warren W. Gay).<br/>
Source code for the book is found at : https://github.com/ve3wwg/stm32f103c8t6.


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
├── opencm3_freertos.c<br/>
├── port.c<br/>
├── queue.c<br/>
├── README.md.c<br/>
├── stream_buffer.c<br/>
├── tasks.c<br/>
└── timers.c<br/>
