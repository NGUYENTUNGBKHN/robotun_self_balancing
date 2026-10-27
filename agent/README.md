# RoboTun MCU Template Generator Agent

A standardized framework and automation toolkit for scaffolding new microcontroller (MCU) firmware targets for the **RoboTun Self-Balancing Robot** project.

---

## Purpose

When expanding the robot to a new microcontroller target (e.g., STM32F4, STM32G4, STM32H7, or NXP/RP2040), this agent package ensures that:
1. **Architectural Parity**: The new MCU folder has the identical modular layout as `stm32f7` and `stm32f1`.
2. **Standardized Coding Style**: Every file contains the uniform Doxygen header block, section banners, and English documentation.
3. **Dual Build Systems**: Both CMake cross-compilation and GNU Makefiles are provided out of the box with flash utilities.
4. **Instant AI Generation**: Any AI agent (such as Antigravity, Gemini, Claude, or GPT) can read `AGENT_PROMPT.md` and generate a fully compliant codebase on demand.

---

## Directory Structure

```
agent/
├── README.md                      # Framework overview and user guide (this file)
├── AGENT_PROMPT.md                # System prompt for AI agents to generate code
├── CODING_STANDARDS.md            # Coding conventions, banners, and Doxygen rules
├── ARCHITECTURE_SPECIFICATION.md  # Detailed technical specifications of all 7 layers
│
├── generator/
│   └── generate_template.py       # Python CLI tool to scaffold new MCU targets
│
└── templates/                     # Master blueprint templates for all files
    ├── CMakeLists_root.txt.tmpl
    ├── arm-none-eabi.cmake.tmpl
    ├── memory_map.h.tmpl
    ├── ace_bsp.h.tmpl
    ├── common.h.tmpl
    ├── reg_util.h.tmpl
    ├── app.ld.tmpl
    ├── boot.ld.tmpl
    ├── main.c.tmpl
    ├── boot_main.c.tmpl
    ├── startup.c.tmpl
    ├── boot_startup.c.tmpl
    ├── debug_uart.c.tmpl
    ├── tx_initialize_low_level.S.tmpl
    └── stepper.c.tmpl
```

---

## How to Ask the AI Agent to Create a New Template

Whenever you want to add a new MCU target, you can simply ask the AI agent in English or Vietnamese:

> **Example Prompt**:
> *"Please create a template for STM32F407 using the rules in the `agent` folder. Target specs: Cortex-M4, FPU fpv4-sp-d16, 168 MHz SYSCLK, 1024 KB Flash, 192 KB SRAM."*

The AI agent will read `agent/AGENT_PROMPT.md` and generate the complete folder under `4_CODE/<target>/`.

---

## Automated CLI Generator

You can also generate any new MCU target instantly using the Python generator script:

```bash
# Basic usage for STM32F407
python agent/generator/generate_template.py --target stm32f4 --mcu STM32F407xx --core cortex-m4 --fpu fpv4-sp-d16 --clock 168000000 --flash 1024K --ram 192K

# Usage for STM32G474 (Cortex-M4, 170 MHz)
python agent/generator/generate_template.py --target stm32g4 --mcu STM32G474xx --core cortex-m4 --fpu fpv4-sp-d16 --clock 170000000 --flash 512K --ram 128K

# Usage for STM32H743 (Cortex-M7, 480 MHz)
python agent/generator/generate_template.py --target stm32h7 --mcu STM32H743xx --core cortex-m7 --fpu fpv5-d16 --clock 480000000 --flash 2048K --ram 1024K
```

The script will automatically:
1. Create all directory trees (`Application`, `Bootloader`, `Cmake`, `Common`, `Middleware`, `OS`, `SDK`).
2. Calculate flash partitioning (Bootloader, Download area, Application).
3. Generate root `CMakeLists.txt`, `Makefile`, and toolchain configurations.
4. Populate `main.c`, `boot_main.c`, `stepper.c`, `debug_uart.c`, and `tx_initialize_low_level.S`.
5. Apply the official author headers, banners, and English documentation.
