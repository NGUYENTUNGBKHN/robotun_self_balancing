/*
****************************************************************************************************************
 * @file       boot_startup.c
 * @brief      C startup and interrupt vector table for STM32F103xB Bootloader
 * @date       2026/10/25
 * @author     [Gentantun] (nguyenthanhtung8196@gmail.com)
 * @details    Sets up initial stack, copies initialised .data from Flash to SRAM,
 *             zeroes .bss, executes SystemInit(), and jumps to boot_main().
 * @ref        stm32f7/Bootloader/boot_startup.c
 * @copyright  Copyright (c) 2026 RoboTun
****************************************************************************************************************
*/
/***************************************************************************************************************
**                                                   INCLUDES
***************************************************************************************************************/
#include <stdint.h>
#include "system_stm32f1xx.h"
#include "stm32f1xx.h"

/***************************************************************************************************************
**                                        EXTERNAL VARIABLE DECLARATIONS
***************************************************************************************************************/
extern uint32_t _estack;
extern uint32_t _sidata;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;

/***************************************************************************************************************
**                                        EXTERNAL FUNCTION PROTOTYPES
***************************************************************************************************************/
extern int boot_main(void);

/***************************************************************************************************************
**                                         INTERNAL FUNCTION PROTOTYPES
***************************************************************************************************************/
void Reset_Handler(void);
void default_handler(void);

/***************************************************************************************************************
**                                             FUNCTION DEFINITIONS
***************************************************************************************************************/

/**
 * @brief  Bootloader reset handler.
 */
void Reset_Handler(void)
{
    volatile uint32_t *src  = &_sidata;
    volatile uint32_t *dst  = &_sdata;
    volatile uint32_t *end  = &_edata;
    volatile uint32_t *bss  = &_sbss;
    volatile uint32_t *bend = &_ebss;

    /* CMSIS hardware system init */
    SystemInit();

    /* Copy initialised data from FLASH to SRAM */
    while (dst < end)
    {
        *dst++ = *src++;
    }

    /* Zero-fill .bss section */
    while (bss < bend)
    {
        *bss++ = 0U;
    }

    /* Jump to bootloader main */
    boot_main();

    /* Should never reach here */
    while (1)
    {
    }
}

/**
 * @brief  Default handler for unhandled exceptions.
 */
void default_handler(void)
{
    while (1)
    {
    }
}

/* Cortex-M3 Core Exceptions */
void NMI_IRQHandler(void)                   __attribute__((weak, alias("default_handler")));
void HardFault_IRQHandler(void)             __attribute__((weak, alias("default_handler")));
void MemManage_IRQHandler(void)             __attribute__((weak, alias("default_handler")));
void BusFault_IRQHandler(void)              __attribute__((weak, alias("default_handler")));
void UsageFault_IRQHandler(void)            __attribute__((weak, alias("default_handler")));
void SVC_IRQHandler(void)                   __attribute__((weak, alias("default_handler")));
void DebugMonitor_IRQHandler(void)          __attribute__((weak, alias("default_handler")));
void PendSV_IRQHandler(void)                __attribute__((weak, alias("default_handler")));
void SysTick_IRQHandler(void)               __attribute__((weak, alias("default_handler")));

/* STM32F103xB External Interrupts */
void WWDG_IRQHandler(void)                  __attribute__((weak, alias("default_handler")));
void PVD_IRQHandler(void)                   __attribute__((weak, alias("default_handler")));
void TAMPER_IRQHandler(void)                __attribute__((weak, alias("default_handler")));
void RTC_IRQHandler(void)                   __attribute__((weak, alias("default_handler")));
void FLASH_IRQHandler(void)                 __attribute__((weak, alias("default_handler")));
void RCC_IRQHandler(void)                   __attribute__((weak, alias("default_handler")));
void EXTI0_IRQHandler(void)                 __attribute__((weak, alias("default_handler")));
void EXTI1_IRQHandler(void)                 __attribute__((weak, alias("default_handler")));
void EXTI2_IRQHandler(void)                 __attribute__((weak, alias("default_handler")));
void EXTI3_IRQHandler(void)                 __attribute__((weak, alias("default_handler")));
void EXTI4_IRQHandler(void)                 __attribute__((weak, alias("default_handler")));
void DMA1_Channel1_IRQHandler(void)         __attribute__((weak, alias("default_handler")));
void DMA1_Channel2_IRQHandler(void)         __attribute__((weak, alias("default_handler")));
void DMA1_Channel3_IRQHandler(void)         __attribute__((weak, alias("default_handler")));
void DMA1_Channel4_IRQHandler(void)         __attribute__((weak, alias("default_handler")));
void DMA1_Channel5_IRQHandler(void)         __attribute__((weak, alias("default_handler")));
void DMA1_Channel6_IRQHandler(void)         __attribute__((weak, alias("default_handler")));
void DMA1_Channel7_IRQHandler(void)         __attribute__((weak, alias("default_handler")));
void ADC1_2_IRQHandler(void)                __attribute__((weak, alias("default_handler")));
void USB_HP_CAN1_TX_IRQHandler(void)        __attribute__((weak, alias("default_handler")));
void USB_LP_CAN1_RX0_IRQHandler(void)       __attribute__((weak, alias("default_handler")));
void CAN1_RX1_IRQHandler(void)              __attribute__((weak, alias("default_handler")));
void CAN1_SCE_IRQHandler(void)              __attribute__((weak, alias("default_handler")));
void EXTI9_5_IRQHandler(void)               __attribute__((weak, alias("default_handler")));
void TIM1_BRK_IRQHandler(void)              __attribute__((weak, alias("default_handler")));
void TIM1_UP_IRQHandler(void)               __attribute__((weak, alias("default_handler")));
void TIM1_TRG_COM_IRQHandler(void)          __attribute__((weak, alias("default_handler")));
void TIM1_CC_IRQHandler(void)               __attribute__((weak, alias("default_handler")));
void TIM2_IRQHandler(void)                  __attribute__((weak, alias("default_handler")));
void TIM3_IRQHandler(void)                  __attribute__((weak, alias("default_handler")));
void TIM4_IRQHandler(void)                  __attribute__((weak, alias("default_handler")));
void I2C1_EV_IRQHandler(void)               __attribute__((weak, alias("default_handler")));
void I2C1_ER_IRQHandler(void)               __attribute__((weak, alias("default_handler")));
void I2C2_EV_IRQHandler(void)               __attribute__((weak, alias("default_handler")));
void I2C2_ER_IRQHandler(void)               __attribute__((weak, alias("default_handler")));
void SPI1_IRQHandler(void)                  __attribute__((weak, alias("default_handler")));
void SPI2_IRQHandler(void)                  __attribute__((weak, alias("default_handler")));
void USART1_IRQHandler(void)                __attribute__((weak, alias("default_handler")));
void USART2_IRQHandler(void)                __attribute__((weak, alias("default_handler")));
void USART3_IRQHandler(void)                __attribute__((weak, alias("default_handler")));
void EXTI15_10_IRQHandler(void)             __attribute__((weak, alias("default_handler")));
void RTC_Alarm_IRQHandler(void)             __attribute__((weak, alias("default_handler")));
void USBWakeUp_IRQHandler(void)             __attribute__((weak, alias("default_handler")));

/***************************************************************************************************************
**                                           Bootloader Vector Table
***************************************************************************************************************/
__attribute__((section(".isr_vector")))
uint32_t *_isr_vector[] =
{
    (uint32_t *)&_estack,
    (uint32_t *)Reset_Handler,
    (uint32_t *)NMI_IRQHandler,
    (uint32_t *)HardFault_IRQHandler,
    (uint32_t *)MemManage_IRQHandler,
    (uint32_t *)BusFault_IRQHandler,
    (uint32_t *)UsageFault_IRQHandler,
    (uint32_t *)0,
    (uint32_t *)0,
    (uint32_t *)0,
    (uint32_t *)0,
    (uint32_t *)SVC_IRQHandler,
    (uint32_t *)DebugMonitor_IRQHandler,
    (uint32_t *)0,
    (uint32_t *)PendSV_IRQHandler,
    (uint32_t *)SysTick_IRQHandler,

    /* External Interrupts */
    (uint32_t *)WWDG_IRQHandler,
    (uint32_t *)PVD_IRQHandler,
    (uint32_t *)TAMPER_IRQHandler,
    (uint32_t *)RTC_IRQHandler,
    (uint32_t *)FLASH_IRQHandler,
    (uint32_t *)RCC_IRQHandler,
    (uint32_t *)EXTI0_IRQHandler,
    (uint32_t *)EXTI1_IRQHandler,
    (uint32_t *)EXTI2_IRQHandler,
    (uint32_t *)EXTI3_IRQHandler,
    (uint32_t *)EXTI4_IRQHandler,
    (uint32_t *)DMA1_Channel1_IRQHandler,
    (uint32_t *)DMA1_Channel2_IRQHandler,
    (uint32_t *)DMA1_Channel3_IRQHandler,
    (uint32_t *)DMA1_Channel4_IRQHandler,
    (uint32_t *)DMA1_Channel5_IRQHandler,
    (uint32_t *)DMA1_Channel6_IRQHandler,
    (uint32_t *)DMA1_Channel7_IRQHandler,
    (uint32_t *)ADC1_2_IRQHandler,
    (uint32_t *)USB_HP_CAN1_TX_IRQHandler,
    (uint32_t *)USB_LP_CAN1_RX0_IRQHandler,
    (uint32_t *)CAN1_RX1_IRQHandler,
    (uint32_t *)CAN1_SCE_IRQHandler,
    (uint32_t *)EXTI9_5_IRQHandler,
    (uint32_t *)TIM1_BRK_IRQHandler,
    (uint32_t *)TIM1_UP_IRQHandler,
    (uint32_t *)TIM1_TRG_COM_IRQHandler,
    (uint32_t *)TIM1_CC_IRQHandler,
    (uint32_t *)TIM2_IRQHandler,
    (uint32_t *)TIM3_IRQHandler,
    (uint32_t *)TIM4_IRQHandler,
    (uint32_t *)I2C1_EV_IRQHandler,
    (uint32_t *)I2C1_ER_IRQHandler,
    (uint32_t *)I2C2_EV_IRQHandler,
    (uint32_t *)I2C2_ER_IRQHandler,
    (uint32_t *)SPI1_IRQHandler,
    (uint32_t *)SPI2_IRQHandler,
    (uint32_t *)USART1_IRQHandler,
    (uint32_t *)USART2_IRQHandler,
    (uint32_t *)USART3_IRQHandler,
    (uint32_t *)EXTI15_10_IRQHandler,
    (uint32_t *)RTC_Alarm_IRQHandler,
    (uint32_t *)USBWakeUp_IRQHandler,
};

/***************************************************************************************************************
**                                                End of file
***************************************************************************************************************/
