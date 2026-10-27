/*
****************************************************************************************************************
 * @file       boot_it.c
 * @brief      Bootloader exception implementations for STM32F103xB
 * @date       2026/10/25
 * @author     [Gentantun] (nguyenthanhtung8196@gmail.com)
 * @details    Provides core fault exception bodies and SysTick tick increment for the bootloader.
 * @ref        stm32f7/Bootloader/boot_it.c
 * @copyright  Copyright (c) 2026 RoboTun
****************************************************************************************************************
*/
/***************************************************************************************************************
**                                                   INCLUDES
***************************************************************************************************************/
#include "boot_it.h"
#include "boot_main.h"

/***************************************************************************************************************
**                                             FUNCTION DEFINITIONS
***************************************************************************************************************/

void NMI_IRQHandler(void)
{
}

void HardFault_IRQHandler(void)
{
    while (1)
    {
    }
}

void MemManage_IRQHandler(void)
{
    while (1)
    {
    }
}

void BusFault_IRQHandler(void)
{
    while (1)
    {
    }
}

void UsageFault_IRQHandler(void)
{
    while (1)
    {
    }
}

void SVC_IRQHandler(void)
{
}

void DebugMonitor_IRQHandler(void)
{
}

void PendSV_IRQHandler(void)
{
}

void SysTick_IRQHandler(void)
{
    HAL_IncTick();
}

/***************************************************************************************************************
**                                                End of file
***************************************************************************************************************/
