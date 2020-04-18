# Common FreeRTOS
CFILES_FREERTOS_BASE =  event_groups.c
CFILES_FREERTOS_BASE += list.c
CFILES_FREERTOS_BASE += port.c
CFILES_FREERTOS_BASE += queue.c
CFILES_FREERTOS_BASE += stream_buffer.c
CFILES_FREERTOS_BASE += tasks.c
CFILES_FREERTOS_BASE += timers.c

# To use libopencm3 with FreeRTOS on Cortex-M3 platform
CFILES_FREERTOS_OPENCM3 = opencm3_freertos.c

# Shall be included in the project Makefile
CFILES_FREERTOS = $(CFILES_FREERTOS_BASE) $(CFILES_FREERTOS_OPENCM3)

# Heap management options, inlude one of these in the project Makefile
CFILES_FREERTOS_HEAP_1 = heap_1.c
CFILES_FREERTOS_HEAP_2 = heap_2.c
CFILES_FREERTOS_HEAP_3 = heap_3.c
CFILES_FREERTOS_HEAP_4 = heap_4.c
CFILES_FREERTOS_HEAP_5 = heap_5.c
