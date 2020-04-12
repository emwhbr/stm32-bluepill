PROJECT = proj1
BUILD_DIR = bin

CFILES = proj1.c
CFILES += api.c
AFILES += api-asm.S

INCLUDES += $(patsubst %,-I%, . $(SHARED_DIR))

include ../rules.mk
