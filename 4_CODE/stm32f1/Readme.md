# STM32F103xB Firmware Project

Firmware suite for STM32F103xB (ARM Cortex-M3, 72 MHz) self-balancing robot controller, featuring modular architecture with dual-partition bootloader, Azure RTOS ThreadX, and stepper motor motion control.

---

## Architecture Overview

```
stm32f1/
├── Application/        # Application binary running on ThreadX RTOS
│   ├── app.ld          # Application linker script (Flash: 0x08010000, 64 KB)
│   ├── app_it.c/h      # Exception vectors (SysTick & PendSV for ThreadX)
│   ├── main.c/h        # App entry point, ThreadX kernel & stepper tasks
│   ├── debug_uart.c/h  # Register-level USART1 driver (PA9 @ 115200)
│   ├── startup.c       # Vector table & Reset_Handler
│   ├── tx_initialize_low_level.S  # Low-level ThreadX initialization
│   ├── syscalls.c      # Newlib stubs with printf redirection
│   ├── sysmem.c        # _sbrk heap allocator
│   ├── CMakeLists.txt  # Application target build rules
│   └── Makefile        # Application target makefile
│
├── Bootloader/         # Second-stage Bootloader & DFU state machine
│   ├── boot.ld         # Bootloader linker script (Flash: 0x08000000, 32 KB)
│   ├── boot_it.c/h     # Core exception handlers
│   ├── boot_main.c/h   # Verification & jump to Application
│   ├── boot_startup.c  # Vector table & Reset_Handler
│   ├── debug_uart.c/h  # Register-level USART1 driver
│   ├── boot_process/   # DFU finite state machine (IDLE, START, DATA, END)
│   ├── CMakeLists.txt  # Bootloader target build rules
│   └── Makefile        # Bootloader target makefile
│
├── Cmake/              # CMake toolchain configuration
│   └── arm-none-eabi.cmake # Cross-compilation toolchain definition
│
├── Common/             # Shared project headers
│   ├── ace_bsp.h       # Pinouts, clocks, peripheral definitions
│   ├── common.h        # TRACE_INFO and debug logging macros
│   ├── memory_map.h    # Flash memory partitions & entry vectors
│   └── reg_util.h      # Register bitwise operation macros
│
├── Middleware/         # Hardware abstraction drivers
│   └── Stepper/        # Stepper motor driver (TIM3 CH1 PWM on PA6, DIR on PB5)
│       ├── stepper.c
│       └── stepper.h
│
├── OS/                 # Real-time Operating System
│   └── threadx/        # Azure RTOS ThreadX Cortex-M3 port
│       ├── common/     # Architecture-independent kernel sources
│       └── ports/      # Cortex-M3 GNU assembly port & context switcher
│
├── SDK/                # Silicon vendor libraries
│   ├── Core/           # CMSIS Core headers (Cortex-M3)
│   ├── Device/         # CMSIS STM32F103xB peripheral register headers
│   └── Driver/hal/     # STM32F1xx Hardware Abstraction Layer (HAL)
│
└── CMakeLists.txt      # Root CMake configuration
```

---

## Memory Map

| Partition | Start Address | End Address | Size | Description |
|:---|:---|:---|:---|:---|
| **Bootloader** | `0x08000000` | `0x08007FFF` | 32 KB | Bootloader binary & DFU recovery engine |
| **Download (DWL)** | `0x08008000` | `0x0800FFFF` | 32 KB | Firmware staging area & DFU entry vector |
| **Application** | `0x08010000` | `0x0801FFFF` | 64 KB | Main user application (ThreadX RTOS) |
| **Internal SRAM** | `0x20000000` | `0x20004FFF` | 20 KB | Stack, Heap, Data & ThreadX pools |

---

## Build Instructions

### 1. Configure CMake
```bash
cmake -B _build -DCMAKE_TOOLCHAIN_FILE=Cmake/arm-none-eabi.cmake -G "MinGW Makefiles"
```

### 2. Build STM32 HAL Library
```bash
cmake --build _build --target stm32_lib -j4
```

### 3. Build Azure RTOS ThreadX (Cortex-M3)
```bash
cmake --build _build --target tx_m3 -j4
```

### 4. Build Bootloader
```bash
cmake --build _build --target boot -j4
```

### 5. Build Application
```bash
cmake --build _build --target app -j4
```

### 6. Build All Targets
```bash
cmake --build _build -j4
```

---

## Flashing Instructions

Flashing is configured to use **SEGGER J-Link** (`-c port=JLINK`) via `STM32_Programmer_CLI.exe`:

```bash
# Flash Bootloader to 0x08000000 via J-Link
cmake --build _build --target flash_boot

# Flash Application to 0x08010000 via J-Link
cmake --build _build --target flash_app

# Erase Entire Flash via J-Link
cmake --build _build --target erase_flash
```

> **Note:** To switch to ST-LINK if ever needed, reconfigure CMake with:
> ```bash
> cmake -B _build -DPROBE_PORT=SWD
> ```

```bash
# Hardware reset : wired nRST (JLINK) with board
COMMAND STM32_Programmer_CLI.exe -c port=${PROBE_PORT} reset=HWrst
        -w ${CMAKE_CURRENT_BINARY_DIR}/boot.bin 0x08000000 -v -hardRst
# Not have nRST, use hotplug + soft reset:
COMMAND STM32_Programmer_CLI.exe -c port=${PROBE_PORT} mode=HOTPLUG
        -w ${CMAKE_CURRENT_BINARY_DIR}/boot.bin 0x08000000 -v -rst
# Remove reset, only program + verify and reset with your hand.
COMMAND STM32_Programmer_CLI.exe -c port=${PROBE_PORT}
        -w ${CMAKE_CURRENT_BINARY_DIR}/boot.bin 0x08000000 -v
# Remain -rst but not show error 
COMMAND STM32_Programmer_CLI.exe -c port=${PROBE_PORT}
        -w ${CMAKE_CURRENT_BINARY_DIR}/boot.bin 0x08000000 -v -rst
        || cmd /c exit 0
```
