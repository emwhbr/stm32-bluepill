PROJECT = blink
BUILD_DIR = bin

CFILES = blink.c
CFILES += uart.c
CFILES += systick.c

INCLUDES += $(patsubst %,-I%, . $(SHARED_DIR))

include ../rules.mk
