/*
****************************************************************************************************************
 * @file       boot_it.h
 * @brief      Bootloader interrupt and core exception handlers for STM32F103xB
 * @date       2026/10/25
 * @author     [Gentantun] (nguyenthanhtung8196@gmail.com)
 * @details    Declares core Cortex-M3 exception handler prototypes for the bootloader.
 * @ref        stm32f7/Bootloader/boot_it.h
 * @copyright  Copyright (c) 2026 RoboTun
****************************************************************************************************************
*/
#ifndef _BOOT_IT_H_
#define _BOOT_IT_H_
#ifdef __cplusplus
extern "C"
{
#endif

/***************************************************************************************************************
**                                         EXTERNAL FUNCTION PROTOTYPES
***************************************************************************************************************/
void NMI_IRQHandler(void);
void HardFault_IRQHandler(void);
void MemManage_IRQHandler(void);
void BusFault_IRQHandler(void);
void UsageFault_IRQHandler(void);
void SVC_IRQHandler(void);
void DebugMonitor_IRQHandler(void);
void PendSV_IRQHandler(void);
void SysTick_IRQHandler(void);

#ifdef __cplusplus
}
#endif
#endif /* _BOOT_IT_H_ */
