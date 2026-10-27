# Application — builds app executable (STM32F103xB, Cortex-M3, 72 MHz)
# Mirrors the structure of stm32f7/Application/CMakeLists.txt

# --------------------------------------------------------------------------
# Sources
# --------------------------------------------------------------------------
file(GLOB APP_C_SRC     "${CMAKE_CURRENT_SOURCE_DIR}/*.c")
file(GLOB APP_ASM_S_SRC "${CMAKE_CURRENT_SOURCE_DIR}/*.S")   # uppercase .S (pre-processed)
file(GLOB APP_ASM_s_SRC "${CMAKE_CURRENT_SOURCE_DIR}/*.s")   # lowercase .s

add_executable(app
    ${APP_C_SRC}
    ${CMAKE_SOURCE_DIR}/Middleware/Stepper/stepper.c
    ${APP_ASM_S_SRC}
    ${APP_ASM_s_SRC}
)

# --------------------------------------------------------------------------
# Include directories
# --------------------------------------------------------------------------
target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/
    ${CMAKE_SOURCE_DIR}/Common
    ${CMAKE_SOURCE_DIR}/Middleware/Stepper
)

# --------------------------------------------------------------------------
# Compile flags (Cortex-M3, STM32F103xB)
# --------------------------------------------------------------------------
target_compile_options(app PRIVATE
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
set(APP_LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/app.ld")

target_link_options(app PRIVATE
    -mcpu=cortex-m3 -mthumb -mabi=aapcs
    -O2 -g3
    --specs=nano.specs
    -Wl,--gc-sections
    -T${APP_LINKER_SCRIPT}
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/app.map
)

set_target_properties(app PROPERTIES OUTPUT_NAME "app" SUFFIX ".elf")

# --------------------------------------------------------------------------
# Libraries (stm32_lib, tx_m3)
# --------------------------------------------------------------------------
target_link_libraries(app PRIVATE
    stm32_lib
    tx_m3
)

# --------------------------------------------------------------------------
# Post-build: generate binary and Intel HEX images
# --------------------------------------------------------------------------
add_custom_command(TARGET app POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary  $<TARGET_FILE:app>
            ${CMAKE_CURRENT_BINARY_DIR}/app.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex    $<TARGET_FILE:app>
            ${CMAKE_CURRENT_BINARY_DIR}/app.hex
    COMMENT "Generating app.bin and app.hex"
)

# --------------------------------------------------------------------------
# Flash targets (via STM32_Programmer_CLI with J-Link)
# --------------------------------------------------------------------------
if(NOT DEFINED PROBE_PORT)
    set(PROBE_PORT "JLINK")
endif()

# Flash application to internal FLASH partition at 0x08010000
add_custom_target(flash_app
    DEPENDS app
    COMMAND STM32_Programmer_CLI.exe -c port=${PROBE_PORT} reset=HWrst
        -w ${CMAKE_CURRENT_BINARY_DIR}/app.bin 0x08010000 -v -hardRst
    COMMENT "Flashing app.bin to internal flash at 0x08010000 via ${PROBE_PORT}"
)

# Erase all flash
add_custom_target(erase_flash
    COMMAND STM32_Programmer_CLI.exe -c port=${PROBE_PORT} -e all
    COMMENT "Erasing all flash via ${PROBE_PORT}"
)
