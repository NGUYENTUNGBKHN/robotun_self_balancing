# RoboTun Firmware Coding Standards & Guidelines

This document outlines the software engineering principles, styling guidelines, and naming conventions for all firmware developed for the RoboTun Self-Balancing Robot platform.

---

## 1. File Organization and Structure

### 1.1 Directory Tree Structure
Every MCU platform folder must follow this exact organization:
```
<mcu_target>/
├── Application/        # Application source code and RTOS threads
├── Bootloader/         # Second-stage bootloader and DFU FSM
├── Cmake/              # Toolchain definition files
├── Common/             # Shared headers, registers, and BSP pin definitions
├── Middleware/         # Platform-independent driver abstractions (Stepper, IMU)
├── OS/                 # RTOS kernel and architecture port files
├── SDK/                # Silicon vendor libraries (CMSIS Core, Device, HAL)
├── CMakeLists.txt      # Root CMake project configuration
└── Readme.md           # Build and flash documentation
```

### 1.2 File Naming Conventions
- All source files must be in lowercase `snake_case`: e.g., `main.c`, `debug_uart.c`, `boot_startup.c`.
- Header files share the same basename: `main.h`, `debug_uart.h`, `stepper.h`.
- Assembly files:
  - Upper-case `.S` for pre-processed assembly files (e.g., `tx_initialize_low_level.S`).
  - Lower-case `.s` for pure assembly files.
- Linker scripts: `app.ld`, `boot.ld`.

---

## 2. Header and Banner Standards

### 2.1 File Header Block
Every `.c` and `.h` file must begin with the standard Doxygen block:
```c
/*
****************************************************************************************************************
 * @file       <filename>
 * @brief      <concise description of the file's role>
 * @date       YYYY/MM/DD
 * @author     [Gentantun] (nguyenthanhtung8196@gmail.com)
 * @details    <technical details, register mapping, clock assumptions>
 * @ref        <reference template or hardware manual>
 * @copyright  Copyright (c) 2026 RoboTun
****************************************************************************************************************
*/
```

### 2.2 Section Banners
Separate functional zones using 112-character banner lines:
```c
/***************************************************************************************************************
**                                                   INCLUDES
***************************************************************************************************************/

/***************************************************************************************************************
**                                          INTERNAL MACRO DEFINITIONS
***************************************************************************************************************/

/***************************************************************************************************************
**                                         EXTERNAL VARIABLE DECLARATIONS
***************************************************************************************************************/

/***************************************************************************************************************
**                                        INTERNAL VARIABLE DEFINITIONS
***************************************************************************************************************/

/***************************************************************************************************************
**                                         INTERNAL FUNCTION PROTOTYPES
***************************************************************************************************************/

/***************************************************************************************************************
**                                             FUNCTION DEFINITIONS
***************************************************************************************************************/

/***************************************************************************************************************
**                                                End of file
***************************************************************************************************************/
```

### 2.3 Include Guards
All header files must use standard `#ifndef` guards with uppercase names:
```c
#ifndef _STEPPER_H_
#define _STEPPER_H_
#ifdef __cplusplus
extern "C"
{
#endif

/* Code */

#ifdef __cplusplus
}
#endif
#endif /* _STEPPER_H_ */
```

---

## 3. Naming and Typing Conventions

### 3.1 Data Types
- Never use unqualified `int`, `long`, `short`, or `char` for numeric variables.
- Always use standard `<stdint.h>` types:
  - `uint8_t`, `uint16_t`, `uint32_t`, `uint64_t`
  - `int8_t`, `int16_t`, `int32_t`, `int64_t`
  - `bool` from `<stdbool.h>`
- Use `size_t` for memory sizes and buffer lengths.
- Pointers to hardware registers must use `volatile`.

### 3.2 Functions
- Function names must use lower-case `snake_case` prefixed by their module name:
  - `stepper_init()`
  - `stepper_move_steps()`
  - `debug_uart_putchar()`
- Functions private to a translation unit must be marked `static`.
- Function parameters not modified within the function should be marked `const`.
- Functions taking no arguments must explicitly declare `(void)`.

### 3.3 Variables
- Local and global variables: `snake_case` (e.g., `completed_steps`, `target_steps`).
- Global variables should be kept to a minimum and prefixed with `g_` if exposed.
- File-scope variables must be marked `static`.

### 3.4 Macros and Constants
- Use uppercase `SNAKE_CASE`: `APP_START_ADDRESS`, `STEP_TIMER`, `DEMO_STACK_SIZE`.
- Numeric constants should include explicit type suffixes: `0U`, `1000UL`, `0x08010000U`.
- Wrap macro definitions in parentheses:
  ```c
  #define REG_SET_BIT(reg, pos)       ((reg) |= (1U << (pos)))
  ```

---

## 4. Error Handling and Defensive Coding

1. **Parameter Validation**: Validate all pointers against `NULL` before dereferencing.
2. **Explicit Unused Parameters**: Cast intentionally unused parameters with `(void)param;` to silence `-Wunused-parameter`.
3. **Array Bounds**: Never perform unchecked array accesses.
4. **Infinite Loops in Fault Handlers**: Unhandled exceptions (HardFault, BusFault, UsageFault) must enter an explicit `while(1)` for debugger inspection.
5. **Print Redirection**: Route standard `printf()` through `debug_uart_putchar()` in `syscalls.c`.
