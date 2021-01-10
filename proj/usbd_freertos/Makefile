PROJECT = usbd_freertos

CFILES = usbd_freertos.c
CFILES += debug_pin.c
CFILES += uart.c uart_stdio.c
CFILES += $(CFILES_FREERTOS) $(CFILES_FREERTOS_HEAP_4)

include ../../freertos.mk
include ../../rules.mk
