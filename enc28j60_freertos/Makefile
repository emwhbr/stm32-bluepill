PROJECT = enc28j60_freertos

CFILES = enc28j60_freertos.c enc28j60.c
CFILES += uart.c uart_stdio.c dwt_delay.c
CFILES += $(CFILES_FREERTOS) $(CFILES_FREERTOS_HEAP_4)

include ../freertos.mk
include ../rules.mk
