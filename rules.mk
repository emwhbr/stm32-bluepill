DEVICE = stm32f103c8t6

OPENCM3_DIR=../libopencm3
include $(OPENCM3_DIR)/mk/genlink-config.mk

SHARED_DIR = ../shared
VPATH += $(SHARED_DIR)

OOCD_FILE = board/stm32f4discovery.cfg

### OPTIONAL ###
# INCLUDES - fully formed -I paths, if you want extra, eg -I../shared
# BUILD_DIR - defaults to bin, should set this if you are building multiarch
# OPT - full -O flag, defaults to -Os
# CSTD - defaults -std=c99
# CXXSTD - no default.
# OOCD_INTERFACE - eg stlink-v2
# OOCD_TARGET - eg stm32f4x
#    both only used if you use the "make flash" target.
# OOCD_FILE - eg my.openocd.cfg
#    This overrides interface/target above, and is used as just -f FILE
### TODO/FIXME/notes ###
# No support for stylecheck.
# No support for BMP/texane/random flash methods, no plans either
# No support for magically finding the library.
# C++ hasn't been actually tested with this..... sorry bout that. ;)
# Second expansion/secondary not set, add this if you need them.

BUILD_DIR ?= bin
OPT ?= -Os
CSTD ?= -std=c99

# Be silent per default, but 'make V=1' will show all compiler calls.
# If you're insane, V=99 will print out all sorts of things.
V ?=0
ifeq ($(V),0)
Q	 := @
NULL := 2>/dev/null
endif

# Tool paths.
PREFIX	?= arm-none-eabi-
CC	    = $(PREFIX)gcc
LD	    = $(PREFIX)gcc
OBJCOPY	= $(PREFIX)objcopy
OBJDUMP	= $(PREFIX)objdump
SIZE    = $(PREFIX)size
OOCD	?= openocd

OPENCM3_INC = $(OPENCM3_DIR)/include

# Inclusion of library header files
INCLUDES += $(patsubst %,-I%, . $(OPENCM3_INC) )

OBJS = $(CFILES:%.c=$(BUILD_DIR)/%.o)
OBJS += $(AFILES:%.S=$(BUILD_DIR)/%.o)
GENERATED_BINS = $(PROJECT).elf $(PROJECT).bin $(PROJECT).map $(PROJECT).list $(PROJECT).lss

TGT_CPPFLAGS += -MD
TGT_CPPFLAGS += -Wall -Wundef $(INCLUDES)
TGT_CPPFLAGS += $(INCLUDES) $(OPENCM3_DEFS)

TGT_CFLAGS += $(OPT) $(CSTD) -ggdb3
TGT_CFLAGS += $(ARCH_FLAGS)
TGT_CFLAGS += -fno-common
TGT_CFLAGS += -ffunction-sections -fdata-sections
TGT_CFLAGS += -Wextra -Wshadow -Wno-unused-variable -Wimplicit-function-declaration
TGT_CFLAGS += -Wredundant-decls -Wstrict-prototypes -Wmissing-prototypes

TGT_CXXFLAGS += $(OPT) $(CXXSTD) -ggdb3
TGT_CXXFLAGS += $(ARCH_FLAGS)
TGT_CXXFLAGS += -fno-common
TGT_CXXFLAGS += -ffunction-sections -fdata-sections
TGT_CXXFLAGS += -Wextra -Wshadow -Wredundant-decls  -Weffc++

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

%.flash: %.elf
	@printf "  FLASH\t$<\n"
ifeq (,$(OOCD_FILE))
	$(Q)(echo "halt; program $(realpath $(*).elf) verify reset" | nc -4 localhost 4444 2>/dev/null) || \
		$(OOCD) -f interface/$(OOCD_INTERFACE).cfg \
		-f target/$(OOCD_TARGET).cfg \
		-c "program $(realpath $(*).elf) verify reset exit" \
		$(NULL)
else
	$(Q)(echo "halt; program $(realpath $(*).elf) verify reset" | nc -4 localhost 4444 2>/dev/null) || \
		$(OOCD) -f $(OOCD_FILE) \
		-c "program $(realpath $(*).elf) verify reset exit" \
		$(NULL)
endif

clean:
	@printf "  CLEAN\t$(PROJECT)\n"
	$(Q)rm -rf $(BUILD_DIR) $(GENERATED_BINS)

.PHONY: all clean flash
-include $(OBJS:.o=.d)

