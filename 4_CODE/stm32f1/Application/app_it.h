/*
****************************************************************************************************************
 * @file       app_it.h
 * @brief      Interrupt handler declarations for the STM32F1 Application layer
 * @date       2026/10/25
 * @author     [Gentantun] (nguyenthanhtung8196@gmail.com)
 * @details    Include this header in any translation unit that needs to forward-declare
 *             the application-level exception/interrupt handlers.
 * @ref        stm32f7/Application/app_it.h
 * @copyright  Copyright (c) 2026 RoboTun
****************************************************************************************************************
*/
#ifndef _APP_IT_H_
#define _APP_IT_H_
#ifdef __cplusplus
extern "C"
{
#endif

/* Exception handlers implemented in app_it.c */
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
#endif /* _APP_IT_H_ */
