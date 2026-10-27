/**
 ******************************************************************************
 * @file      sysmem.c
 * @brief     Newlib _sbrk() heap allocator for STM32F1 bare-metal targets
 * @details   Grows the heap upward from the end of .bss toward the reserved
 *            stack area.  Protects the MSP stack by checking against _estack.
 *
 * @note      This file is identical to the stm32f7 version because _sbrk()
 *            relies only on linker symbols (_end, _estack, _Min_Stack_Size)
 *            which are defined in app.ld.
 *
 * Copyright (c) 2024 STMicroelectronics.  All rights reserved.
 * Licensed under the terms found in the LICENSE file.
 ******************************************************************************
 */

/* Includes */
#include <errno.h>
#include <stdint.h>

/**
 * @brief  Pointer to the current high watermark of heap usage.
 *         Initialised to NULL so the first call sets it to &_end.
 */
static uint8_t *__sbrk_heap_end = NULL;

/**
 * @brief  _sbrk() — called by malloc/free to extend the heap.
 *
 * Memory layout (low → high address in SRAM):
 *   .data | .bss | heap → … ← MSP stack (reserved) | _estack
 *
 * @param  incr  Number of bytes to add (positive) or subtract (negative).
 * @return Pointer to the previous heap end on success, (void*)-1 on OOM.
 */
void *_sbrk(ptrdiff_t incr)
{
    extern uint8_t  _end;             /* Set by linker: first free byte after .bss */
    extern uint8_t  _estack;          /* Set by linker: top of SRAM               */
    extern uint32_t _Min_Stack_Size;  /* Set by linker: reserved stack size        */

    const uint32_t stack_limit = (uint32_t)&_estack - (uint32_t)&_Min_Stack_Size;
    const uint8_t *max_heap    = (uint8_t *)stack_limit;
    uint8_t *prev_heap_end;

    /* Initialise heap pointer on first call */
    if (NULL == __sbrk_heap_end)
    {
        __sbrk_heap_end = &_end;
    }

    /* Guard: refuse to grow into the reserved MSP stack */
    if (__sbrk_heap_end + incr > max_heap)
    {
        errno = ENOMEM;
        return (void *)-1;
    }

    prev_heap_end    = __sbrk_heap_end;
    __sbrk_heap_end += incr;

    return (void *)prev_heap_end;
}
