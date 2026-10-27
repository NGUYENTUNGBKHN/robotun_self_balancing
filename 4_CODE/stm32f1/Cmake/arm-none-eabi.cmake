# Toolchain file for ARM Cortex-M bare-metal with arm-none-eabi-gcc
# Usage: cmake -B _cmake_build -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake

set(CMAKE_SYSTEM_NAME      Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Disable compiler feature tests (no OS to run test executables)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# --------------------------------------------------------------------------
# Toolchain prefix — reads TOOLCHAIN_TUNG env var (same as Makefile.windows)
# --------------------------------------------------------------------------
if(DEFINED ENV{TOOLCHAIN_TUNG})
    # Convert Windows backslashes to forward slashes to avoid CMake parse errors
    file(TO_CMAKE_PATH "$ENV{TOOLCHAIN_TUNG}" TOOLCHAIN_PREFIX)
    # Ensure trailing slash
    string(REGEX REPLACE "([^/])$" "\\1/" TOOLCHAIN_PREFIX "${TOOLCHAIN_PREFIX}")
else()
    set(TOOLCHAIN_PREFIX "")
endif()

set(TRIPLE "arm-none-eabi")

# On Windows, executables need the .exe suffix for CMake to validate them
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

# Search programs only on host
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
