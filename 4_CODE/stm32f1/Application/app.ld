/*
****************************************************************************************************************
 * Linker script for STM32F103xB Application partition
 * 128 KB Flash total:
 *   - 0x08000000 - 0x08007FFF : Bootloader (32 KB)
 *   - 0x08008000 - 0x0800FFFF : Download staging area (32 KB)
 *   - 0x08010000 - 0x0801FFFF : Application partition (64 KB)
 * 20 KB SRAM (0x20000000 - 0x20004FFF)
****************************************************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)

/* Highest address of the user mode stack */
_estack = ORIGIN(RAM) + LENGTH(RAM);   /* end of SRAM */

_Min_Heap_Size  = 0x200;   /* required heap  */
_Min_Stack_Size = 0x400;   /* required stack */

MEMORY
{
  RAM   (xrw) : ORIGIN = 0x20000000, LENGTH = 20K
  FLASH (rx)  : ORIGIN = 0x08010000, LENGTH = 64K   /* Application partition */
}

/* Sections */
SECTIONS
{
  /* ISR vector table — must be at the very start of FLASH */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH

  /* Code and read-only constant data */
  .text :
  {
    . = ALIGN(4);
    *(.text)            /* .text sections (code)            */
    *(.text*)           /* .text* sections (code)           */
    *(.glue_7)          /* glue arm-to-thumb code           */
    *(.glue_7t)         /* glue thumb-to-arm code           */
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
    _etext = .;         /* global symbol at end of code     */
  } >FLASH

  /* Read-only constant data */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)          /* constants, string literals, etc. */
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  /* Used by the startup code to initialise .data from FLASH */
  _sidata = LOADADDR(.data);

  /* Initialised data placed in SRAM at run-time, stored in FLASH at load-time */
  .data :
  {
    . = ALIGN(4);
    _sdata = .;         /* global symbol: data start in SRAM */
    *(.data)
    *(.data*)
    *(.RamFunc)         /* functions that must run from RAM  */
    *(.RamFunc*)
    . = ALIGN(4);
    _edata = .;         /* global symbol: data end in SRAM   */
  } >RAM AT> FLASH

  /* Uninitialised data — zeroed by startup */
  . = ALIGN(4);
  .bss :
  {
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } >RAM

  /* Heap and stack guard region */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end  = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM

  /* DWARF debug sections */
  .ARM.attributes 0 : { *(.ARM.attributes) }

  __RAM_segment_used_end__ = .;
}
