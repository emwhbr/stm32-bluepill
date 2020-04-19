# stm32-bluepill


## General
Exploring the STM32 MCU by using the "Blue Pill" STM32F103C8T6.

This repository includes different projects based on [LibOpenCM3](https://libopencm3.org/) and [FreeRTOS](https://www.freertos.org/).<br/>

The layout and build structure was inspired by the contents found at:<br/>
https://github.com/libopencm3/libopencm3-template.git

## FreeRTOS
The use of FreeRTOS in the projects is described [here](FreeRTOS/README.md).

## Dependencies
All projects have been built and tested with:

* Linux Mint 19 Tara (x86_64)
* GNU Arm Embedded Toolchain, Version 9-2019-q4-major
* OpenOCD, 0.10.0
* SEGGER J-Link EDU, V6.55a

## Build projects
Setup the environment:
```
> source ./scripts/setup_environment.sh
```
Build all projects, including libopencm3:
```
> make all
```
Build and manage an individual project:
```
> make -C blink_freertos clean
> make -C blink_freertos all
> make -C blink_freertos reset
> make -C blink_freertos flash
```
The reset and flash options will make use of OpenOCD and J-Link to control the target.
