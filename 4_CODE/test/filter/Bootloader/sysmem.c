/*
****************************************************************************************************************
 * @file       sysmem.c
 * @brief      Minimal _sbrk implementation for STM32F1 Bootloader dynamic memory allocation
 * @date       2026/10/25
 * @author     [Gentantun] (nguyenthanhtung8196@gmail.com)
 * @details    Provides heap management boundary checks against _Min_Heap_Size and _estack.
 * @ref        stm32f7/Bootloader/sysmem.c
 * @copyright  Copyright (c) 2026 RoboTun
****************************************************************************************************************
*/
/***************************************************************************************************************
**                                                   INCLUDES
***************************************************************************************************************/
#include <stdint.h>
#include <errno.h>

/***************************************************************************************************************
**                                        EXTERNAL VARIABLE DECLARATIONS
***************************************************************************************************************/
extern uint8_t end;
extern uint8_t _estack;
extern uint32_t _Min_Stack_Size;

/***************************************************************************************************************
**                                             FUNCTION DEFINITIONS
***************************************************************************************************************/

/**
 * @brief  _sbrk allocates memory to the newlib heap pointer.
 * @param  incr  Number of bytes to increment heap.
 * @return void* Pointer to allocated memory, or (void *)-1 on out of memory.
 */
void *_sbrk(ptrdiff_t incr)
{
    static uint8_t *__sbrk_heap_end = NULL;
    uint8_t *prev_heap_end;
    const uint8_t *max_stack_limit = (uint8_t *)&_estack - (uint32_t)&_Min_Stack_Size;

    if (__sbrk_heap_end == NULL)
    {
        __sbrk_heap_end = &end;
    }

    prev_heap_end = __sbrk_heap_end;

    if (__sbrk_heap_end + incr > max_stack_limit)
    {
        errno = ENOMEM;
        return (void *)-1;
    }

    __sbrk_heap_end += incr;
    return (void *)prev_heap_end;
}

/***************************************************************************************************************
**                                                End of file
***************************************************************************************************************/
