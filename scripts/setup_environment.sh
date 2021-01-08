#!/bin/sh

# ARM GNU Toolchain
ARM=/opt/arm/gcc-arm-none-eabi-9-2019-q4-major

# SEGGER JLINK
SEGGER=/opt/SEGGER/JLink

# STMicroelectronics ST-LINK
STLINK=/opt/stlink

# OPENOCD
OPENOCD=/opt/openocd

export PATH=${ARM}/bin:${SEGGER}:${STLINK}/bin:${OPENOCD}/bin:${PATH}
