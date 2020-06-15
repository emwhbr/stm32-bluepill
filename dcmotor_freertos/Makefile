PROJECT = dcmotor_freertos

CFILES = dcmotor_freertos.c
CFILES += adc.c motor_ctrl.c motor_encoder.c
CFILES += uart.c uart_stdio.c
CFILES += $(CFILES_FREERTOS) $(CFILES_FREERTOS_HEAP_4)

include ../freertos.mk
include ../rules.mk
