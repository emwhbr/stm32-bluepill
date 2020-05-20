PROJECT = alive_freertosip

CFILES = alive_freertosip.c
CFILES += app_stub.c
CFILES += NetworkInterface.c
CFILES += enc28j60.c
CFILES += uart.c uart_stdio.c dwt_delay.c
CFILES += $(CFILES_FREERTOS) $(CFILES_FREERTOS_HEAP_4)
CFILES += $(CFILES_FREERTOS_PLUS_TCP)

include ../freertos.mk
include ../freertos-plus-tcp.mk
include ../rules.mk
