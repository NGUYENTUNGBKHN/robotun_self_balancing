# RoboTun MCU Firmware Architecture Specification

This document details the technical specification for all 7 architectural layers in the RoboTun Self-Balancing Robot codebase.

---

## 1. Application Layer (`Application/`)

### 1.1 Purpose
The Application layer contains the primary robot control logic, state machines, sensor processing tasks, and actuator controllers executed under the Azure RTOS ThreadX operating system.

### 1.2 Key Components
- **`main.c` / `main.h`**:
  - Initializes HAL, System Clock (PLL), user LED, debug UART, and stepper drivers.
  - Calls `tx_kernel_enter()`.
  - Implements `tx_application_define()` to create byte pools and threads.
  - Hosts Thread 0 (motion control loop) and Thread 1 (telemetry/housekeeping).
- **`app_it.c` / `app_it.h`**:
  - Hooks `PendSV_IRQHandler` into `__tx_PendSVHandler()`.
  - Hooks `SysTick_IRQHandler` into `__tx_SysTickHandler()` followed by `HAL_IncTick()`.
- **`tx_initialize_low_level.S`**:
  - Configures SysTick reload register for 100 Hz RTOS tick rate.
  - Relocates VTOR register to `_isr_vector`.
  - Configures SVC and PendSV interrupt priorities to lowest (`0xFF`).
  - Sets up `_tx_initialize_unused_memory` pointing to RAM after `.bss` and heap.
- **`startup.c`**:
  - Defines the full vector table (`.isr_vector`).
  - Implements `Reset_Handler` to copy initialised `.data`, zero `.bss`, call `SystemInit()`, and branch to `main()`.
- **`syscalls.c` / `sysmem.c`**:
  - Minimal Newlib runtime integration redirecting `_write` to `debug_uart_putchar()`.
- **`app.ld`**:
  - Places code into `FLASH` at `APP_START_ADDRESS`.
  - Defines `__RAM_segment_used_end__ = .;` at end of RAM allocation.

---

## 2. Bootloader Layer (`Bootloader/`)

### 2.1 Purpose
A robust, standalone second-stage bootloader responsible for hardware integrity verification, DFU update handling, and jumping to the application.

### 2.2 Boot Jump Sequence
When booting, `boot_main.c` evaluates the application partition at `APP_START_ADDRESS`:
```c
uint32_t app_msp = *(volatile uint32_t *)APP_START_ADDRESS;
if ((app_msp & 0x2FFE0000U) == 0x20000000U)
{
    /* 1. Read Reset_Handler address */
    uint32_t app_reset = *(volatile uint32_t *)(APP_START_ADDRESS + 4U);

    /* 2. Reset Core Registers */
    __set_CONTROL(0U);
    __set_PRIMASK(0U);
    __set_BASEPRI(0U);
    __set_FAULTMASK(0U);

    /* 3. Set Main Stack Pointer */
    __set_MSP(app_msp);

    /* 4. Relocate Vector Table (VTOR) */
    SCB->VTOR = APP_START_ADDRESS;

    /* 5. Jump to Application Reset_Handler */
    ((void (*)(void))app_reset)();
}
```

### 2.3 DFU Finite State Machine (`boot_process/`)
- **`BL_STATE_IDLE`**: Awaiting commands from host interface.
- **`BL_STATE_DWL_START`**: Handshake verification and flash erase.
- **`BL_STATE_DWL_DATA`**: Streaming binary payload writing.
- **`BL_STATE_DWL_END`**: CRC32 integrity check and system reboot.
- **`.entry_dwl`**: Fixed function pointer table placed in the download sector.

---

## 3. Common Layer (`Common/`)

### 3.1 `memory_map.h`
Defines memory boundaries for internal Flash and SRAM:
- `BOOT_START_ADDRESS`: Base of Flash (`0x08000000`).
- `BOOT_SIZE`: Bootloader partition size.
- `DWL_START_ADDRESS`: Staging partition base.
- `DWL_SIZE`: Staging partition size.
- `APP_START_ADDRESS`: Main application base (`BOOT_START_ADDRESS + BOOT_SIZE + DWL_SIZE`).
- `APP_SIZE`: Remaining flash allocated to application.

### 3.2 `ace_bsp.h`
Maps physical hardware pins, clock frequencies, and peripherals:
- LED pin and port definitions.
- Debug UART peripheral instance and baud rate.
- Stepper motor STEP/DIR timer and GPIO mappings.

### 3.3 `common.h`
Logging macros active when `DEBUG` is defined:
- `TRACE_INFO(format, ...)`: Formatted log with function name and line number.
- `ERROR(format, ...)`: Error output macro.

### 3.4 `reg_util.h`
Macro library for direct register bit manipulation:
- `REG_WRITE(reg, val)`
- `REG_READ(reg)`
- `REG_SET_BIT(reg, pos)`
- `REG_CLR_BIT(reg, pos)`
- `REG_READ_BIT(reg, pos)`
- `REG_MODIFY(reg, clrmask, setmask)`

---

## 4. Middleware Layer (`Middleware/`)

### 4.1 Stepper Motor Driver (`Middleware/Stepper/`)
Encapsulates step pulse generation via hardware timer PWM and direction control:
- **`stepper_init()`**: Configures timer time base to 1 us resolution and sets 50% PWM duty cycle.
- **`stepper_set_direction(dir)`**: Updates DIR GPIO (`STEPPER_FORWARD` or `STEPPER_REVERSE`).
- **`stepper_set_frequency(hz)`**: Recomputes timer auto-reload register: `period = 1000000U / hz`.
- **`stepper_move_steps(steps, dir, hz)`**: Arms timer compare interrupt (CC1) to count discrete steps and automatically stops when target count is reached.
- **`stepper_is_busy()`**: Returns non-zero while a step sequence is in progress.

---

## 5. OS Layer (`OS/threadx/`)

Contains Microsoft Azure RTOS ThreadX:
- **`common/`**: Platform-independent RTOS services (threads, byte pools, mutexes, event flags, timers).
- **`ports/<architecture>/gnu/`**: Low-level assembly context switcher (`tx_thread_schedule.S`, `tx_timer_interrupt.S`).

---

## 6. SDK Layer (`SDK/`)

Vendor hardware support libraries:
- **`Core/Include`**: CMSIS Core headers (`core_cm3.h`, `core_cm4.h`, `core_cm7.h`).
- **`Device/Inc`**: Register definitions (`stm32f103xb.h`, `system_stm32f1xx.h`).
- **`Driver/hal/`**: STMicroelectronics HAL drivers compiled into `stm32_lib.a`.

---

## 7. CMake Build System (`Cmake/` & `CMakeLists.txt`)

- Cross-compilation configured via `Cmake/arm-none-eabi.cmake`.
- Subdirectories: `SDK` (stm32_lib), `OS/threadx` (tx_<core>), `Bootloader` (boot), `Application` (app).
- Post-build steps generate `.bin` and `.hex`.
- Custom flashing targets invoke `STM32_Programmer_CLI.exe` configured with `-c port=JLINK` by default (can be overridden with `-DPROBE_PORT=SWD`).
