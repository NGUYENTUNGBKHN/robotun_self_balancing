/*
****************************************************************************************************************
 * Linker script for STM32F103xB Bootloader
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
_estack = ORIGIN(RAM) + LENGTH(RAM);

_Min_Heap_Size  = 0x200;
_Min_Stack_Size = 0x400;

MEMORY
{
  RAM       (xrw) : ORIGIN = 0x20000000, LENGTH = 20K
  FLASH     (rx)  : ORIGIN = 0x08000000, LENGTH = 16K
  DWL_FLASH (rx)  : ORIGIN = 0x08004000, LENGTH = 16K
}

SECTIONS
{
  /* ISR vector table at start of FLASH */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH

  /* Code */
  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)
    KEEP (*(.init))
    KEEP (*(.fini))
    . = ALIGN(4);
    _etext = .;
  } >FLASH

  /* Constant data */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  _sidata = LOADADDR(.data);

  /* Initialised data placed into SRAM */
  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    *(.RamFunc)
    *(.RamFunc*)
    . = ALIGN(4);
    _edata = .;
  } >RAM AT> FLASH

  /* Download entry function table placed in DWL_FLASH */
  .entry_dwl 0x08004000 :
  {
    KEEP(*(.entry_dwl))
  } >DWL_FLASH

  /* Uninitialised data in SRAM */
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

  /* Stack and heap allocation check */
  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end  = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM

  .ARM.attributes 0 : { *(.ARM.attributes) }
  __RAM_segment_used_end__ = .;
}
