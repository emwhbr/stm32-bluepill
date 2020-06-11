######################################################################
#  Top Level: STM32F103C8T6 "Bluepill" Projects
######################################################################

PROJECTS =  blink
PROJECTS += blink_freertos
PROJECTS += stdio_freertos
PROJECTS += ssd1306_freertos
PROJECTS += littlevgl_basic
PROJECTS += enc28j60_freertosip
PROJECTS += irdecode_freertos
PROJECTS += irgui_freertos
PROJECTS += dcmotor_freertos

.PHONY = libopencm3

# Be silent per default, but 'make V=1' will show all compiler calls
ifneq ($(V),1)
Q := @
# Do not print "Entering directory ..."
MAKEFLAGS += --no-print-directory
endif

all: libopencm3
	for d in $(PROJECTS) ; do \
		$(MAKE) -C $$d ; \
	done

clean: clean_libopencm3
	for d in $(PROJECTS) ; do \
		$(MAKE) -C $$d clean ; \
	done

libopencm3: libopencm3/lib/libopencm3_stm32f1.a

libopencm3/lib/libopencm3_stm32f1.a:
	$(MAKE) -C libopencm3 TARGETS=stm32/f1

clean_libopencm3:
	rm -f libopencm3/lib/libopencm3_stm32f1.a
	-$(MAKE) -$(MAKEFLAGS) -C ./libopencm3 clean

