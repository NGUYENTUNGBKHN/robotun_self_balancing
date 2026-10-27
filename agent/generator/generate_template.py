#!/usr/bin/env python3
"""
RoboTun MCU Firmware Template Generator
Author: [Gentantun] (nguyenthanhtung8196@gmail.com)
Copyright (c) 2026 RoboTun

Automatically scaffolds a new MCU project directory adhering strictly to the
RoboTun 7-layer architecture, Doxygen standards, section banners, and ThreadX RTOS integration.
"""

import os
import sys
import argparse
from datetime import datetime

BANNER_TEMPLATE = """/***************************************************************************************************************
**                                                   {title}
***************************************************************************************************************/"""

DOXYGEN_HEADER = """/*
****************************************************************************************************************
 * @file       {filename}
 * @brief      {brief}
 * @date       {date}
 * @author     [Gentantun] (nguyenthanhtung8196@gmail.com)
 * @details    {details}
 * @ref        {ref}
 * @copyright  Copyright (c) {year} RoboTun
****************************************************************************************************************
*/"""

def parse_size(size_str: str) -> int:
    """Parses size string like '64K', '128K', '1M', '1024K' into bytes."""
    size_str = size_str.strip().upper()
    if size_str.endswith("K"):
        return int(size_str[:-1]) * 1024
    elif size_str.endswith("M"):
        return int(size_str[:-1]) * 1024 * 1024
    return int(size_str)

def format_hex(val: int) -> str:
    return f"0x{val:08X}"

def main():
    parser = argparse.ArgumentParser(description="RoboTun MCU Firmware Template Generator")
    parser.add_argument("--target", required=True, help="Target folder name, e.g. stm32f4")
    parser.add_argument("--mcu", required=True, help="MCU preprocessor define, e.g. STM32F407xx")
    parser.add_argument("--core", default="cortex-m4", choices=["cortex-m0", "cortex-m0plus", "cortex-m3", "cortex-m4", "cortex-m7", "cortex-m33"])
    parser.add_argument("--fpu", default="fpv4-sp-d16", help="FPU flag, e.g. fpv4-sp-d16 or none")
    parser.add_argument("--clock", type=int, default=168000000, help="SYSCLK frequency in Hz, e.g. 168000000")
    parser.add_argument("--flash", default="1024K", help="Total Flash size, e.g. 1024K, 512K, 128K")
    parser.add_argument("--ram", default="192K", help="Total SRAM size, e.g. 192K, 128K, 20K")
    parser.add_argument("--outdir", default=None, help="Root directory for 4_CODE (defaults to workspace 4_CODE)")

    args = parser.parse_args()

    date_str = datetime.now().strftime("%Y/%m/%d")
    year_str = datetime.now().strftime("%Y")

    total_flash = parse_size(args.flash)
    total_ram = parse_size(args.ram)

    # Calculate partitions
    if total_flash <= 128 * 1024:
        boot_size = 32 * 1024
        dwl_size = 32 * 1024
    elif total_flash <= 512 * 1024:
        boot_size = 32 * 1024
        dwl_size = 64 * 1024
    else:
        boot_size = 32 * 1024
        dwl_size = 64 * 1024

    app_start = 0x08000000 + boot_size + dwl_size
    app_size = total_flash - (boot_size + dwl_size)

    # FPU flags
    if args.fpu.lower() in ["none", "no", ""]:
        fpu_compile_flag = ""
        fpu_cmake_flag = ""
    else:
        fpu_compile_flag = f"-mfloat-abi=hard -mfpu={args.fpu}"
        fpu_cmake_flag = f'set(MCU_FPU_FLAGS "-mfloat-abi=hard -mfpu={args.fpu}")'

    # Determine base workspace directory
    if args.outdir:
        base_dir = os.path.abspath(args.outdir)
    else:
        # Search for 4_CODE relative to script
        script_dir = os.path.dirname(os.path.abspath(__file__))
        repo_root = os.path.abspath(os.path.join(script_dir, "..", ".."))
        base_dir = os.path.join(repo_root, "4_CODE")

    target_dir = os.path.join(base_dir, args.target)
    print(f"[INFO] Generating RoboTun MCU Template for '{args.target}' ({args.mcu}) at:")
    print(f"       {target_dir}")

    # Subdirectories
    subdirs = [
        "Application",
        "Bootloader",
        "Bootloader/boot_process",
        "Bootloader/boot_process/operation",
        "Cmake",
        "Common",
        "Middleware",
        "Middleware/Stepper",
        "OS",
        "OS/threadx",
        "SDK",
    ]
    for sd in subdirs:
        os.makedirs(os.path.join(target_dir, sd), exist_ok=True)

    # 1. Root CMakeLists.txt
    root_cmake = f"""cmake_minimum_required(VERSION 3.20)

# Must be set BEFORE project() so toolchain is picked up
# Usage: cmake -B _build -DCMAKE_TOOLCHAIN_FILE=Cmake/arm-none-eabi.cmake -G "MinGW Makefiles"

project(ACE C ASM)

# --------------------------------------------------------------------------
# Global MCU flags ({args.mcu} {args.core.upper()})
# --------------------------------------------------------------------------
set(MCU_CPU_FLAGS "-mcpu={args.core} -mthumb -mabi=aapcs")
{fpu_cmake_flag}
set(MCU_DEFINES   "-D{args.mcu}")

set(MCU_FLAGS "${{MCU_CPU_FLAGS}} ${{MCU_FPU_FLAGS}}" CACHE STRING "MCU compiler flags")

find_program(CMAKE_OBJCOPY arm-none-eabi-objcopy HINTS $ENV{{TOOLCHAIN_TUNG}})

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
"""
    with open(os.path.join(target_dir, "CMakeLists.txt"), "w", encoding="utf-8") as f:
        f.write(root_cmake)

    # 2. Cmake/arm-none-eabi.cmake
    toolchain_cmake = """# Toolchain file for ARM Cortex-M bare-metal with arm-none-eabi-gcc
# Usage: cmake -B _build -DCMAKE_TOOLCHAIN_FILE=Cmake/arm-none-eabi.cmake

set(CMAKE_SYSTEM_NAME      Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

if(DEFINED ENV{TOOLCHAIN_TUNG})
    file(TO_CMAKE_PATH "$ENV{TOOLCHAIN_TUNG}" TOOLCHAIN_PREFIX)
    string(REGEX REPLACE "([^/])$" "\\\\1/" TOOLCHAIN_PREFIX "${TOOLCHAIN_PREFIX}")
else()
    set(TOOLCHAIN_PREFIX "")
endif()

set(TRIPLE "arm-none-eabi")

if(WIN32)
    set(EXE_SUFFIX ".exe")
else()
    set(EXE_SUFFIX "")
endif()

set(CMAKE_C_COMPILER   "${TOOLCHAIN_PREFIX}${TRIPLE}-gcc${EXE_SUFFIX}")
set(CMAKE_CXX_COMPILER "${TOOLCHAIN_PREFIX}${TRIPLE}-g++${EXE_SUFFIX}")
set(CMAKE_ASM_COMPILER "${TOOLCHAIN_PREFIX}${TRIPLE}-gcc${EXE_SUFFIX}")
set(CMAKE_AR           "${TOOLCHAIN_PREFIX}${TRIPLE}-ar${EXE_SUFFIX}"     CACHE FILEPATH "")
set(CMAKE_RANLIB       "${TOOLCHAIN_PREFIX}${TRIPLE}-ranlib${EXE_SUFFIX}" CACHE FILEPATH "")
set(CMAKE_OBJCOPY      "${TOOLCHAIN_PREFIX}${TRIPLE}-objcopy${EXE_SUFFIX}" CACHE FILEPATH "")
set(CMAKE_OBJDUMP      "${TOOLCHAIN_PREFIX}${TRIPLE}-objdump${EXE_SUFFIX}" CACHE FILEPATH "")
set(CMAKE_SIZE         "${TOOLCHAIN_PREFIX}${TRIPLE}-size${EXE_SUFFIX}"    CACHE FILEPATH "")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
"""
    with open(os.path.join(target_dir, "Cmake", "arm-none-eabi.cmake"), "w", encoding="utf-8") as f:
        f.write(toolchain_cmake)

    # 3. Common/memory_map.h
    header_memmap = DOXYGEN_HEADER.format(
        filename="memory_map.h",
        brief=f"Memory layout definitions for {args.mcu}",
        date=date_str,
        details=f"Defines Bootloader, Download staging, and Application partitions for {args.flash} Flash and {args.ram} SRAM.",
        ref="agent/templates/memory_map.h.tmpl",
        year=year_str
    )
    content_memmap = f"""{header_memmap}
#ifndef _MEMORY_MAP_H_
#define _MEMORY_MAP_H_
#ifdef __cplusplus
extern "C"
{{
#endif

{BANNER_TEMPLATE.format(title="INTERNAL MACRO DEFINITIONS")}
#define FUNC(x)                     (*((void (**)(void))(x)))

#define ST_MEMORY_START_ADDRESS     0x00000000U
#define ST_MEMORY_END_ADDRESS       0xFFFFFFFFU

#define ST_FLASH_BASE               0x08000000U
#define ST_FLASH_TOTAL_SIZE         {format_hex(total_flash)}U

#define ST_SRAM_BASE                0x20000000U
#define ST_SRAM_TOTAL_SIZE          {format_hex(total_ram)}U

/* Partitions */
#define BOOT_START_ADDRESS          0x08000000U
#define BOOT_SIZE                   {format_hex(boot_size)}U

#define DWL_START_ADDRESS           {format_hex(0x08000000 + boot_size)}U
#define DWL_SIZE                    {format_hex(dwl_size)}U

#define APP_START_ADDRESS           {format_hex(app_start)}U
#define APP_SIZE                    {format_hex(app_size)}U

#define entry_dwl                   FUNC(DWL_START_ADDRESS)
#define entry_dwl2                  FUNC((DWL_START_ADDRESS + 4U))

#ifdef __cplusplus
}}
#endif
#endif /* _MEMORY_MAP_H_ */
"""
    with open(os.path.join(target_dir, "Common", "memory_map.h"), "w", encoding="utf-8") as f:
        f.write(content_memmap)

    # 4. Common/common.h
    header_common = DOXYGEN_HEADER.format(
        filename="common.h",
        brief="Logging and assertion macros",
        date=date_str,
        details="Provides TRACE_INFO and ERROR macros for standardized debug console output.",
        ref="agent/templates/common.h.tmpl",
        year=year_str
    )
    content_common = f"""{header_common}
#ifndef _COMMON_H_
#define _COMMON_H_
#ifdef __cplusplus
extern "C"
{{
#endif

{BANNER_TEMPLATE.format(title="INCLUDES")}
#include <stdio.h>
#include <stdint.h>

{BANNER_TEMPLATE.format(title="INTERNAL MACRO DEFINITIONS")}
#define DEBUG
#if defined(DEBUG)
#define TRACE_INFO(format, ...)     printf("%s (%d) : " format, __FUNCTION__, __LINE__, ##__VA_ARGS__)
#define ERROR(format, ...)          printf("[ERR] %s (%d) : " format, __FUNCTION__, __LINE__, ##__VA_ARGS__)
#else
#define TRACE_INFO(format, ...)
#define ERROR(format, ...)
#endif

#ifdef __cplusplus
}}
#endif
#endif /* _COMMON_H_ */
"""
    with open(os.path.join(target_dir, "Common", "common.h"), "w", encoding="utf-8") as f:
        f.write(content_common)

    # 5. Common/reg_util.h
    header_reg = DOXYGEN_HEADER.format(
        filename="reg_util.h",
        brief="Register bitwise manipulation macros",
        date=date_str,
        details="Safe macros for setting, clearing, reading, and modifying memory-mapped registers.",
        ref="agent/templates/reg_util.h.tmpl",
        year=year_str
    )
    content_reg = f"""{header_reg}
#ifndef _REG_UTIL_H_
#define _REG_UTIL_H_
#ifdef __cplusplus
extern "C"
{{
#endif

{BANNER_TEMPLATE.format(title="INTERNAL MACRO DEFINITIONS")}
#define REG_WRITE(reg, val)                 ((reg) = (val))
#define REG_READ(reg)                       ((reg))
#define REG_SET_BIT(reg, pos)               ((reg) |= (1U << (pos)))
#define REG_CLR_BIT(reg, pos)               ((reg) &= ~(1U << (pos)))
#define REG_READ_BIT(reg, pos)              (((reg) & (1U << (pos))) >> (pos))
#define REG_CLR_VAL(reg, clrmask, pos)      ((reg) &= ~((clrmask) << (pos)))
#define REG_SET_VAL(reg, val, setmask, pos) do {{ \\
                                                REG_CLR_VAL(reg, setmask, pos); \\
                                                ((reg) |= (((val) & (setmask)) << (pos))); \\
                                            }} while(0)
#define REG_MODIFY(reg, clrmask, setmask)   REG_WRITE((reg), (((REG_READ(reg)) & (~(clrmask))) | (setmask)))

#ifdef __cplusplus
}}
#endif
#endif /* _REG_UTIL_H_ */
"""
    with open(os.path.join(target_dir, "Common", "reg_util.h"), "w", encoding="utf-8") as f:
        f.write(content_reg)

    # 6. Readme.md
    readme_content = f"""# {args.mcu} Firmware Project

Firmware suite for {args.mcu} ({args.core}, {args.clock // 1000000} MHz) self-balancing robot controller.

## Partitions
- **Bootloader**: `0x08000000` ({boot_size // 1024} KB)
- **Download**: `{format_hex(0x08000000 + boot_size)}` ({dwl_size // 1024} KB)
- **Application**: `{format_hex(app_start)}` ({app_size // 1024} KB)
- **SRAM**: `0x20000000` ({total_ram // 1024} KB)

## Build Instructions
```bash
cmake -B _build -DCMAKE_TOOLCHAIN_FILE=Cmake/arm-none-eabi.cmake -G "MinGW Makefiles"
cmake --build _build --target stm32_lib -j4
cmake --build _build --target boot -j4
cmake --build _build --target app -j4
cmake --build _build -j4
```

## Flash Commands
```bash
cmake --build _build --target flash_boot
cmake --build _build --target flash_app
```
"""
    with open(os.path.join(target_dir, "Readme.md"), "w", encoding="utf-8") as f:
        f.write(readme_content)

    print(f"[SUCCESS] Scaffolding for {args.target} completed successfully!")

if __name__ == "__main__":
    main()
