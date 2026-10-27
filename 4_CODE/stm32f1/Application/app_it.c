/*
****************************************************************************************************************
 * @file       app_it.c
 * @brief      Application-level Cortex-M3 exception / interrupt handlers for STM32F1
 * @date       2026/10/25
 * @author     [Gentantun] (nguyenthanhtung8196@gmail.com)
 * @details    Implements the core exception handlers required by the Cortex-M3 vector
 *             table.  PendSV and SysTick are hooked into the ThreadX port.
 *             All unused fault handlers spin forever for easy debugging.
 * @ref        stm32f7/Application/app_it.c (original template)
 * @copyright  Copyright (c) 2026 RoboTun
****************************************************************************************************************
*/
/***************************************************************************************************************
**                                                   INCLUDES
***************************************************************************************************************/
#include "app_it.h"
#include "main.h"

/***************************************************************************************************************
**                                        EXTERNAL FUNCTION PROTOTYPES
***************************************************************************************************************/
/* ThreadX Cortex-M3 port handlers (implemented in assembly port files) */
extern void __tx_SysTickHandler(void);
extern void __tx_PendSVHandler(void);

/***************************************************************************************************************
**                                             FUNCTION DEFINITIONS
***************************************************************************************************************/

/**
 * @brief  Non-Maskable Interrupt handler.
 *         Not used in this application — return immediately.
 */
void NMI_IRQHandler(void)
{
}

/**
 * @brief  Hard Fault exception handler.
 *         Spins forever so a debugger can inspect the call stack.
 */
void HardFault_IRQHandler(void)
{
    /* Go to infinite loop when Hard Fault exception occurs */
    while (1)
    {
    }
}

/**
 * @brief  Memory Management fault handler.
 *         Spins forever; indicates an MPU violation (if MPU is configured).
 */
void MemManage_IRQHandler(void)
{
    /* Go to infinite loop when Memory Manage exception occurs */
    while (1)
    {
    }
}

/**
 * @brief  Bus Fault exception handler.
 *         Spins forever; indicates an AHB/APB bus error.
 */
void BusFault_IRQHandler(void)
{
    /* Go to infinite loop when Bus Fault exception occurs */
    while (1)
    {
    }
}

/**
 * @brief  Usage Fault exception handler.
 *         Spins forever; indicates an undefined instruction or illegal state.
 */
void UsageFault_IRQHandler(void)
{
    /* Go to infinite loop when Usage Fault exception occurs */
    while (1)
    {
    }
}

/**
 * @brief  Supervisor Call handler.
 *         ThreadX uses SVC indirectly; this default implementation is empty.
 */
void SVC_IRQHandler(void)
{
}

/**
 * @brief  Debug Monitor exception handler.
 *         Not used by ThreadX; empty implementation.
 */
void DebugMonitor_IRQHandler(void)
{
}

/**
 * @brief  PendSV exception handler.
 *         Delegated to the ThreadX Cortex-M3 port PendSV handler which
 *         performs the context switch.
 */
void PendSV_IRQHandler(void)
{
    __tx_PendSVHandler();
}

/**
 * @brief  SysTick timer interrupt handler.
 *         - Advances the ThreadX internal tick counter.
 *         - Calls HAL_IncTick() so HAL_Delay() and HAL_GetTick() remain functional.
 */
void SysTick_IRQHandler(void)
{
    __tx_SysTickHandler();
    HAL_IncTick();
}

/***************************************************************************************************************
**                                                End of file
***************************************************************************************************************/
