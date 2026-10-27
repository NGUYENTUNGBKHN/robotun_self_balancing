cmake_minimum_required(VERSION 3.20)

# Must be set BEFORE project() so toolchain is picked up
# Usage: cmake -B _build -DCMAKE_TOOLCHAIN_FILE=Cmake/arm-none-eabi.cmake -G "MinGW Makefiles"

project(ACE C ASM)

# --------------------------------------------------------------------------
# Global MCU flags (STM32F103xB Cortex-M3)
# --------------------------------------------------------------------------
set(MCU_CPU_FLAGS "-mcpu=cortex-m3 -mthumb -mabi=aapcs")
set(MCU_DEFINES   "-DSTM32F103xB")

set(MCU_FLAGS "${MCU_CPU_FLAGS}" CACHE STRING "MCU compiler flags")

# Make CMAKE_OBJCOPY available to subdirectories
find_program(CMAKE_OBJCOPY arm-none-eabi-objcopy HINTS $ENV{TOOLCHAIN_TUNG})

# --------------------------------------------------------------------------
# Flash Probe Interface (Default: JLINK)
# --------------------------------------------------------------------------
set(PROBE_PORT "JLINK" CACHE STRING "Programming probe interface (JLINK, SWD, etc.)")

# --------------------------------------------------------------------------
# Subdirectories — order matters (dependencies first)
# --------------------------------------------------------------------------
add_subdirectory(SDK)
add_subdirectory(OS/threadx)
add_subdirectory(Bootloader)
add_subdirectory(Application)
