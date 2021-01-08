######################################################################
#  Top Level: STM32F103C8T6 "Bluepill" Projects
######################################################################

PROJECT_DIR = "proj"

PROJECTS =  $(PROJECT_DIR)/blink
PROJECTS += $(PROJECT_DIR)/blink_freertos
PROJECTS += $(PROJECT_DIR)/stdio_freertos
PROJECTS += $(PROJECT_DIR)/ssd1306_freertos
PROJECTS += $(PROJECT_DIR)/littlevgl_basic
PROJECTS += $(PROJECT_DIR)/enc28j60_freertosip
PROJECTS += $(PROJECT_DIR)/irdecode_freertos
PROJECTS += $(PROJECT_DIR)/irgui_freertos
PROJECTS += $(PROJECT_DIR)/dcmotor_freertos

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

reset:
# We use an arbitrary project to reset target
	$(MAKE) -C $(PROJECT_DIR)/blink reset

stl_reset:
# We use an arbitrary project to reset target
	$(MAKE) -C $(PROJECT_DIR)/blink stl_reset

erase:
# We use an arbitrary project to erase target
	$(MAKE) -C $(PROJECT_DIR)/blink erase

stl_erase:
# We use an arbitrary project to erase target
	$(MAKE) -C $(PROJECT_DIR)/blink stl_erase

libopencm3: libopencm3/lib/libopencm3_stm32f1.a

libopencm3/lib/libopencm3_stm32f1.a:
	$(MAKE) -C libopencm3 TARGETS=stm32/f1

clean_libopencm3:
	rm -f libopencm3/lib/libopencm3_stm32f1.a
	-$(MAKE) -$(MAKEFLAGS) -C ./libopencm3 clean

