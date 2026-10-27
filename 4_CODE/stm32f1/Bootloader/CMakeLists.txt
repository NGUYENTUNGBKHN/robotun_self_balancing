# Bootloader — builds boot executable (STM32F103xB, Cortex-M3, 72 MHz)
# Equivalent to Bootloader/Makefile

# --------------------------------------------------------------------------
# Sources
# --------------------------------------------------------------------------
file(GLOB BOOT_C_SRC       "${CMAKE_CURRENT_SOURCE_DIR}/*.c")
file(GLOB BOOT_ASM_S_SRC   "${CMAKE_CURRENT_SOURCE_DIR}/*.S")
file(GLOB BOOT_ASM_s_SRC   "${CMAKE_CURRENT_SOURCE_DIR}/*.s")
file(GLOB BOOT_PROC_SRC    "${CMAKE_CURRENT_SOURCE_DIR}/boot_process/*.c")
file(GLOB BOOT_OP_SRC      "${CMAKE_CURRENT_SOURCE_DIR}/boot_process/operation/*.c")

# Exclude boot_startup.s (lowercase) if any
list(FILTER BOOT_ASM_s_SRC EXCLUDE REGEX "boot_startup\\.s$")

add_executable(boot
    ${BOOT_C_SRC}
    ${BOOT_PROC_SRC}
    ${BOOT_OP_SRC}
    ${BOOT_ASM_S_SRC}
    ${BOOT_ASM_s_SRC}
)

# --------------------------------------------------------------------------
# Include directories
# --------------------------------------------------------------------------
target_include_directories(boot PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/boot_process
    ${CMAKE_CURRENT_SOURCE_DIR}/boot_process/operation
    ${CMAKE_SOURCE_DIR}/Common
)

# --------------------------------------------------------------------------
# Compile flags (Cortex-M3, STM32F103xB)
# --------------------------------------------------------------------------
target_compile_options(boot PRIVATE
    -mcpu=cortex-m3 -mthumb -mabi=aapcs
    -O2 -g3
    -ffunction-sections -fdata-sections
    -fno-strict-aliasing -fno-builtin -fshort-enums
    -Wall -Wno-unused-variable
    -DSTM32F103xB
)

# --------------------------------------------------------------------------
# Linker flags
# --------------------------------------------------------------------------
set(BOOT_LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/boot.ld")

target_link_options(boot PRIVATE
    -mcpu=cortex-m3 -mthumb -mabi=aapcs
    -O2 -g3
    --specs=nano.specs
    -Wl,--gc-sections
    -T${BOOT_LINKER_SCRIPT}
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/boot.map
)

set_target_properties(boot PROPERTIES OUTPUT_NAME "boot" SUFFIX ".elf")

# --------------------------------------------------------------------------
# Libraries
# --------------------------------------------------------------------------
target_link_libraries(boot PRIVATE stm32_lib)

# --------------------------------------------------------------------------
# Post-build: generate .bin and .hex
# --------------------------------------------------------------------------
add_custom_command(TARGET boot POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:boot>
            ${CMAKE_CURRENT_BINARY_DIR}/boot.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:boot>
            ${CMAKE_CURRENT_BINARY_DIR}/boot.hex
    COMMENT "Generating boot.bin and boot.hex"
)

# --------------------------------------------------------------------------
# Flash target (via STM32_Programmer_CLI with J-Link)
# --------------------------------------------------------------------------
if(NOT DEFINED PROBE_PORT)
    set(PROBE_PORT "JLINK")
endif()

add_custom_target(flash_boot
    DEPENDS boot
    COMMAND STM32_Programmer_CLI.exe -c port=${PROBE_PORT} reset=HWrst
        -w ${CMAKE_CURRENT_BINARY_DIR}/boot.bin 0x08000000 -v -hardRst
    COMMENT "Flashing boot to internal flash at 0x08000000 via ${PROBE_PORT}"
)
