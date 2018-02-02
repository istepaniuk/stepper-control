cmake_minimum_required(VERSION 3.8)
project(motor C ASM)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_C_COMPILER /usr/bin/arm-none-eabi-gcc)
set(CMAKE_ASM_COMPILER /usr/bin/arm-none-eabi-as)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

set(CMAKE_C_FLAGS "-DSTM32F10X_MD_VL -DUSE_STDPERIPH_DRIVER -mcpu=cortex-m3 -mthumb -g -O0 -Wall -fmessage-length=0 -ffunction-sections -fdata-sections -std=c99")
set(CMAKE_ASM_FLAGS "-mthumb -mcpu=cortex-m3" CACHE INTERNAL "asm compiler flags")
set(CMAKE_EXE_LINKER_FLAGS "-Wl,-gc-sections -T${PROJECT_SOURCE_DIR}/ld/stm32.ld")


include_directories(lib/CMSIS/CM3/CoreSupport)
include_directories(lib/CMSIS/CM3/DeviceSupport/ST/STM32F10x)
include_directories(lib/STM32F10x_StdPeriph_Driver/inc)
include_directories(src)

add_library(HAL
        lib/STM32F10x_StdPeriph_Driver/src/misc.c
        lib/STM32F10x_StdPeriph_Driver/src/stm32f10x_adc.c
        lib/STM32F10x_StdPeriph_Driver/src/stm32f10x_bkp.c
        lib/STM32F10x_StdPeriph_Driver/src/stm32f10x_can.c
        lib/STM32F10x_StdPeriph_Driver/src/stm32f10x_cec.c
        lib/STM32F10x_StdPeriph_Driver/src/stm32f10x_crc.c
        lib/STM32F10x_StdPeriph_Driver/src/stm32f10x_dac.c
        lib/STM32F10x_StdPeriph_Driver/src/stm32f10x_dbgmcu.c
        lib/STM32F10x_StdPeriph_Driver/src/stm32f10x_dma.c
        lib/STM32F10x_StdPeriph_Driver/src/stm32f10x_exti.c
        lib/STM32F10x_StdPeriph_Driver/src/stm32f10x_flash.c
        lib/STM32F10x_StdPeriph_Driver/src/stm32f10x_fsmc.c
        lib/STM32F10x_StdPeriph_Driver/src/stm32f10x_gpio.c
        lib/STM32F10x_StdPeriph_Driver/src/stm32f10x_i2c.c
        lib/STM32F10x_StdPeriph_Driver/src/stm32f10x_iwdg.c
        lib/STM32F10x_StdPeriph_Driver/src/stm32f10x_pwr.c
        lib/STM32F10x_StdPeriph_Driver/src/stm32f10x_rcc.c
        lib/STM32F10x_StdPeriph_Driver/src/stm32f10x_rtc.c
        lib/STM32F10x_StdPeriph_Driver/src/stm32f10x_sdio.c
        lib/STM32F10x_StdPeriph_Driver/src/stm32f10x_spi.c
        lib/STM32F10x_StdPeriph_Driver/src/stm32f10x_tim.c
        lib/STM32F10x_StdPeriph_Driver/src/stm32f10x_usart.c
        lib/STM32F10x_StdPeriph_Driver/src/stm32f10x_wwdg.c)

add_library(CMSIS
        lib/CMSIS/CM3/CoreSupport/core_cm3.c
        lib/CMSIS/CM3/DeviceSupport/ST/STM32F10x/system_stm32f10x.c
        lib/CMSIS/CM3/DeviceSupport/ST/STM32F10x/startup/gcc/startup_stm32f10x_md_vl.S)


set(SOURCE_FILES
        src/interrupts.c
        src/main.c
        src/platform.c
        src/delay.c
        src/leds.c
        src/button.c
        src/usart.c
        src/timer.c test/avr.c)

add_executable(${PROJECT_NAME}.elf ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME}.elf CMSIS)
target_link_libraries(${PROJECT_NAME}.elf HAL)