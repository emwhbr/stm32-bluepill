DEVICE = stm32f103c8t6

OPENCM3_DIR =../libopencm3
include $(OPENCM3_DIR)/mk/genlink-config.mk

FREERTOS_DIR = ../FreeRTOS
VPATH += $(FREERTOS_DIR)

FREERTOS_PLUS_TCP_DIR = ../FreeRTOS-Plus-TCP
VPATH += $(FREERTOS_PLUS_TCP_DIR)

SHARED_DIR = ../shared
VPATH += $(SHARED_DIR)

OOCD_INTERFACE = jlink.cfg
OOCD_TARGET = stm32f1x.cfg
OOCD_PROC = ../scripts/openocd_proc.cfg

BUILD_DIR = bin
OPT = -Os
CSTD = -std=c99

# Be silent per default, but 'make V=1' will show all compiler calls.
# If you're insane, V=99 will print out all sorts of things.
V ?=0
ifeq ($(V),0)
Q	 := @
NULL := 2>/dev/null
endif

# Tool paths.
PREFIX	= arm-none-eabi-
CC	    = $(PREFIX)gcc
LD	    = $(PREFIX)gcc
OBJCOPY	= $(PREFIX)objcopy
OBJDUMP	= $(PREFIX)objdump
SIZE    = $(PREFIX)size
OOCD	= openocd

OPENCM3_INC = $(OPENCM3_DIR)/include
FREERTOS_INC = $(FREERTOS_DIR)/include
FREERTOS_PLUS_TCP_INC = $(FREERTOS_PLUS_TCP_DIR)/include

# Inclusion of library header files
INCLUDES += $(patsubst %,-I%, . $(OPENCM3_INC)  )
INCLUDES += $(patsubst %,-I%, . $(FREERTOS_INC) )
INCLUDES += $(patsubst %,-I%, . $(FREERTOS_PLUS_TCP_INC) )
INCLUDES += $(patsubst %,-I%, . $(SHARED_DIR)   )
INCLUDES += $(patsubst %,-I%, . $(LVGL_DIR) )

OBJS = $(CFILES:%.c=$(BUILD_DIR)/%.o)
OBJS += $(AFILES:%.S=$(BUILD_DIR)/%.o)
GENERATED_BINS = $(PROJECT).elf $(PROJECT).bin $(PROJECT).map $(PROJECT).list $(PROJECT).lss

TGT_CPPFLAGS += -MD
TGT_CPPFLAGS += -Wall -Wundef $(INCLUDES)
TGT_CPPFLAGS += $(INCLUDES) $(OPENCM3_DEFS)

TGT_CFLAGS += $(OPT) $(CSTD) -ggdb3
TGT_CFLAGS += $(ARCH_FLAGS)
TGT_CFLAGS += -Werror
TGT_CFLAGS += -fno-common
TGT_CFLAGS += -ffunction-sections -fdata-sections
TGT_CFLAGS += -Wextra -Wshadow -Wunused-variable -Wimplicit-function-declaration
TGT_CFLAGS += -Wredundant-decls -Wstrict-prototypes -Wmissing-prototypes
TGT_CFLAGS += $(LVGL_CFLAGS)

TGT_CXXFLAGS += $(OPT) $(CXXSTD) -ggdb3
TGT_CXXFLAGS += $(ARCH_FLAGS)
TGT_CXXFLAGS += -Werror
TGT_CXXFLAGS += -fno-common
TGT_CXXFLAGS += -ffunction-sections -fdata-sections
TGT_CXXFLAGS += -Wextra -Wshadow -Wredundant-decls  -Weffc++
TGT_CXXFLAGS += $(LVGL_CFLAGS)

TGT_ASFLAGS += $(OPT) $(ARCH_FLAGS) -ggdb3

TGT_LDFLAGS += -T$(LDSCRIPT) -L$(OPENCM3_DIR)/lib -nostartfiles
TGT_LDFLAGS += $(ARCH_FLAGS)
TGT_LDFLAGS += -specs=nano.specs
TGT_LDFLAGS	+= -Wl,-Map=$(*).map
TGT_LDFLAGS += -Wl,--gc-sections
# OPTIONAL
#TGT_LDFLAGS += -Wl,-Map=$(PROJECT).map
ifeq ($(V),99)
TGT_LDFLAGS += -Wl,--print-gc-sections
endif
ifeq ($(V),1)
TGT_LDFLAGS += -Wl,--trace
endif

# Linker script generator fills this in for us.
ifeq (,$(DEVICE))
LDLIBS += -l$(OPENCM3_LIB)
endif
# nosys is only in newer gcc-arm-embedded...
#LDLIBS += -specs=nosys.specs
LDLIBS += -Wl,--start-group -lc -lgcc -lnosys -Wl,--end-group

# Burn in legacy hell fortran modula pascal yacc idontevenwat
.SUFFIXES:
.SUFFIXES: .c .S .h .o .cxx .elf .bin .list .lss

# Bad make, never *ever* try to get a file out of source control by yourself.
%: %,v
%: RCS/%,v
%: RCS/%
%: s.%
%: SCCS/s.%

all: $(PROJECT).elf $(PROJECT).bin $(PROJECT).list $(PROJECT).lss
flash: $(PROJECT).flash

# We need to disable warnings for some source code in LittleVGL
$(BUILD_DIR)/lv_hal_disp.o    : TGT_CFLAGS += -Wno-unused-parameter
$(BUILD_DIR)/lv_img_decoder.o : TGT_CFLAGS += -Wno-unused-parameter

# We need to disable warnings for some source code in FreeRTOS+TCP
$(BUILD_DIR)/BufferAllocation_2.o     : TGT_CFLAGS += -Wno-redundant-decls
$(BUILD_DIR)/FreeRTOS_ARP.o           : TGT_CFLAGS += -Wno-redundant-decls -Wno-strict-prototypes
$(BUILD_DIR)/FreeRTOS_DHCP.o          : TGT_CFLAGS += -Wno-redundant-decls -Wno-sign-compare
$(BUILD_DIR)/FreeRTOS_DNS.o           : TGT_CFLAGS += -Wno-redundant-decls -Wno-strict-prototypes -Wno-missing-prototypes
$(BUILD_DIR)/FreeRTOS_IP.o            : TGT_CFLAGS += -Wno-redundant-decls -Wno-strict-prototypes -Wno-address-of-packed-member
$(BUILD_DIR)/FreeRTOS_Sockets.o       : TGT_CFLAGS += -Wno-redundant-decls -Wno-sign-compare -Wno-strict-prototypes -Wno-unused-variable
$(BUILD_DIR)/FreeRTOS_Stream_Buffer.o : TGT_CFLAGS += -Wno-redundant-decls
$(BUILD_DIR)/FreeRTOS_TCP_IP.o        : TGT_CFLAGS += -Wno-redundant-decls -Wno-unused-function
$(BUILD_DIR)/FreeRTOS_TCP_WIN.o       : TGT_CFLAGS += -Wno-redundant-decls
$(BUILD_DIR)/FreeRTOS_UDP_IP.o        : TGT_CFLAGS += -Wno-redundant-decls -Wno-strict-prototypes

# Need a special rule to have a bin dir
$(BUILD_DIR)/%.o: %.c
	@printf "  CC\t$<\n"
	@mkdir -p $(dir $@)
	$(Q)$(CC) $(TGT_CFLAGS) $(CFLAGS) $(TGT_CPPFLAGS) $(CPPFLAGS) -o $@ -c $<

$(BUILD_DIR)/%.o: %.cxx
	@printf "  CXX\t$<\n"
	@mkdir -p $(dir $@)
	$(Q)$(CC) $(TGT_CXXFLAGS) $(CXXFLAGS) $(TGT_CPPFLAGS) $(CPPFLAGS) -o $@ -c $<

$(BUILD_DIR)/%.o: %.S
	@printf "  AS\t$<\n"
	@mkdir -p $(dir $@)
	$(Q)$(CC) $(TGT_ASFLAGS) $(ASFLAGS) $(TGT_CPPFLAGS) $(CPPFLAGS) -o $@ -c $<

$(PROJECT).elf: $(OBJS) $(LIBDEPS)
	@printf "  LD\t$@\n"
	$(Q)$(LD) $(TGT_LDFLAGS) $(LDFLAGS) $(OBJS) $(LDLIBS) -o $@
	$(Q)$(SIZE) $(PROJECT).elf

%.bin: %.elf
	@printf "  OBJCOPY\t$@\n"
	$(Q)$(OBJCOPY) -O binary  $< $@

%.lss: %.elf
	@printf "  OBJDUMP\t$@\n"
	$(Q)$(OBJDUMP) -h -S $< > $@

%.list: %.elf
	@printf "  OBJDUMP\t$@\n"
	$(Q)$(OBJDUMP) -S $< > $@

%.flash: %.bin
	@printf "  FLASH TARGET\t$<\n"
	$(Q)$(OOCD) -f interface/$(OOCD_INTERFACE) -c "transport select swd" \
	-f target/$(OOCD_TARGET) -f ${OOCD_PROC} \
	-c "target_program_flash $(realpath $(*).bin)"

reset:
	@printf "  RESET TARGET\n"
	$(Q)$(OOCD) -f interface/$(OOCD_INTERFACE) -c "transport select swd" \
	-f target/$(OOCD_TARGET) -f ${OOCD_PROC} \
	-c "target_reset"

erase:
	@printf "  ERASE FLASH TARGET\n"
	$(Q)$(OOCD) -f interface/$(OOCD_INTERFACE) -c "transport select swd" \
	-f target/$(OOCD_TARGET) -f ${OOCD_PROC} \
	-c "target_erase_flash"

clean:
	@printf "  CLEAN\t$(PROJECT)\n"
	$(Q)rm -rf $(BUILD_DIR) $(GENERATED_BINS)

.PHONY: all clean flash
-include $(OBJS:.o=.d)

