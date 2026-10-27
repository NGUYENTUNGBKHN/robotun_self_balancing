# OS/threadx — builds tx_m3 static library
# Cortex-M3 (ARMv7-M) port for Azure RTOS ThreadX

add_library(tx_m3 STATIC)

# --------------------------------------------------------------------------
# Sources
# --------------------------------------------------------------------------
file(GLOB TX_COMMON_SRC  "${CMAKE_CURRENT_SOURCE_DIR}/common/src/*.c")
file(GLOB TX_PORT_C_SRC  "${CMAKE_CURRENT_SOURCE_DIR}/ports/cortex_m3/gnu/src/*.c")
file(GLOB TX_PORT_S_SRC  "${CMAKE_CURRENT_SOURCE_DIR}/ports/cortex_m3/gnu/src/*.S")

target_sources(tx_m3 PRIVATE
    ${TX_COMMON_SRC}
    ${TX_PORT_C_SRC}
    ${TX_PORT_S_SRC}
)

# --------------------------------------------------------------------------
# Include directories (PUBLIC so Application can see ThreadX headers)
# --------------------------------------------------------------------------
target_include_directories(tx_m3 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/common/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/ports/cortex_m3/gnu/inc
)

# --------------------------------------------------------------------------
# Compile flags (Cortex-M3, no hardware FPU)
# --------------------------------------------------------------------------
target_compile_options(tx_m3 PRIVATE
    -mcpu=cortex-m3 -mthumb -mabi=aapcs
    -O0 -g
    -DSTM32F103xB
)
