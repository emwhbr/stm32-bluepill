PROJECT = proj2
BUILD_DIR = bin

CFILES = proj2.c
CFILES += api.c
AFILES += api-asm.S

INCLUDES += $(patsubst %,-I%, . $(SHARED_DIR))

include ../rules.mk
