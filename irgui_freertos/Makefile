PROJECT = irgui_freertos

CFILES = irgui_freertos.c
CFILES += gui.c
CFILES += uart.c uart_stdio.c ir_nec.c lvgl_ssd1306.c ssd1306.c dwt_delay.c
CFILES += $(CFILES_FREERTOS) $(CFILES_FREERTOS_HEAP_4) $(CFILES_LVGL)

include ../freertos.mk
include ../littlevgl.mk
include ../rules.mk
