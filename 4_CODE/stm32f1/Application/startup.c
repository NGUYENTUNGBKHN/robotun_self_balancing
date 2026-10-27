/*
****************************************************************************************************************
 * @file       startup.c
 * @brief      C startup file for STM32F103xB (Cortex-M3)
 * @date       2026/10/25
 * @author     [Gentantun] (nguyenthanhtung8196@gmail.com)
 * @details    Defines the vector table (.isr_vector section) and Reset_Handler.
 *             Reset_Handler:
 *               1. Calls SystemInit() (CMSIS) to set up the flash prefetch and
 *                  initial clock state.
 *               2. Copies initialised data from FLASH to SRAM (.data section).
 *               3. Zeroes the uninitialised .bss section.
 *               4. Calls main().
 *
 *             All unimplemented IRQ handlers are weakly aliased to
 *             default_handler() which spins forever for easy debugging.
 *
 *             IRQ list matches STM32F103xB (medium-density) vector table.
 *             Reference: RM0008 Rev 21, Table 63.
 *
 * @ref        stm32f7/Application/startup.c (original template)
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
/* Symbols defined by the linker script (app.ld) */
extern uint32_t _estack;    /* Top of SRAM (initial SP value)          */
extern uint32_t _sidata;    /* Load address of .data in FLASH           */
extern uint32_t _sdata;     /* Start of .data in SRAM                  */
extern uint32_t _edata;     /* End   of .data in SRAM                  */
extern uint32_t _sbss;      /* Start of .bss in SRAM                   */
extern uint32_t _ebss;      /* End   of .bss in SRAM                   */

/***************************************************************************************************************
**                                        EXTERNAL FUNCTION PROTOTYPES
***************************************************************************************************************/
extern int main(void);

/***************************************************************************************************************
**                                         INTERNAL FUNCTION PROTOTYPES
***************************************************************************************************************/
void Reset_Handler(void);
void default_handler(void);

/***************************************************************************************************************
**                                             FUNCTION DEFINITIONS
***************************************************************************************************************/

/**
 * @brief  Reset handler — first code executed after reset.
 *         Copies .data from FLASH → SRAM, zeroes .bss, then calls main().
 */
void Reset_Handler(void)
{
    volatile uint32_t *src  = &_sidata;
    volatile uint32_t *dst  = &_sdata;
    volatile uint32_t *end  = &_edata;
    volatile uint32_t *bss  = &_sbss;
    volatile uint32_t *bend = &_ebss;

    /* Call CMSIS SystemInit: configures flash wait states & internal clock */
    SystemInit();

    /* Copy initialised data from FLASH load address to SRAM run address */
    while (dst < end)
    {
        *dst++ = *src++;
    }

    /* Zero-fill the .bss section */
    while (bss < bend)
    {
        *bss++ = 0U;
    }

    /* Jump to application */
    main();

    /* Should never reach here */
    while (1)
    {
    }
}

/**
 * @brief  Default interrupt handler.
 *         All unimplemented IRQ handlers are aliased here.
 *         Spins forever so a debugger can identify the unexpected interrupt.
 */
void default_handler(void)
{
    while (1)
    {
        /* Intentional infinite loop — attach debugger to identify source */
    }
}

/***************************************************************************************************************
**                        Weak aliases — override by defining in app_it.c
***************************************************************************************************************/
/* Core exceptions */
void NMI_IRQHandler(void)          __attribute__((weak, alias("default_handler")));
void HardFault_IRQHandler(void)    __attribute__((weak, alias("default_handler")));
void MemManage_IRQHandler(void)    __attribute__((weak, alias("default_handler")));
void BusFault_IRQHandler(void)     __attribute__((weak, alias("default_handler")));
void UsageFault_IRQHandler(void)   __attribute__((weak, alias("default_handler")));
void SVC_IRQHandler(void)          __attribute__((weak, alias("default_handler")));
void DebugMonitor_IRQHandler(void) __attribute__((weak, alias("default_handler")));
void PendSV_IRQHandler(void)       __attribute__((weak, alias("default_handler")));
void SysTick_IRQHandler(void)      __attribute__((weak, alias("default_handler")));

/* External interrupts — STM32F103xB medium-density (60 IRQs) */
void WWDG_IRQHandler(void)                 __attribute__((weak, alias("default_handler")));
void PVD_IRQHandler(void)                  __attribute__((weak, alias("default_handler")));
void TAMPER_IRQHandler(void)               __attribute__((weak, alias("default_handler")));
void RTC_IRQHandler(void)                  __attribute__((weak, alias("default_handler")));
void FLASH_IRQHandler(void)                __attribute__((weak, alias("default_handler")));
void RCC_IRQHandler(void)                  __attribute__((weak, alias("default_handler")));
void EXTI0_IRQHandler(void)                __attribute__((weak, alias("default_handler")));
void EXTI1_IRQHandler(void)                __attribute__((weak, alias("default_handler")));
void EXTI2_IRQHandler(void)                __attribute__((weak, alias("default_handler")));
void EXTI3_IRQHandler(void)                __attribute__((weak, alias("default_handler")));
void EXTI4_IRQHandler(void)                __attribute__((weak, alias("default_handler")));
void DMA1_Channel1_IRQHandler(void)        __attribute__((weak, alias("default_handler")));
void DMA1_Channel2_IRQHandler(void)        __attribute__((weak, alias("default_handler")));
void DMA1_Channel3_IRQHandler(void)        __attribute__((weak, alias("default_handler")));
void DMA1_Channel4_IRQHandler(void)        __attribute__((weak, alias("default_handler")));
void DMA1_Channel5_IRQHandler(void)        __attribute__((weak, alias("default_handler")));
void DMA1_Channel6_IRQHandler(void)        __attribute__((weak, alias("default_handler")));
void DMA1_Channel7_IRQHandler(void)        __attribute__((weak, alias("default_handler")));
void ADC1_2_IRQHandler(void)               __attribute__((weak, alias("default_handler")));
void USB_HP_CAN1_TX_IRQHandler(void)       __attribute__((weak, alias("default_handler")));
void USB_LP_CAN1_RX0_IRQHandler(void)      __attribute__((weak, alias("default_handler")));
void CAN1_RX1_IRQHandler(void)             __attribute__((weak, alias("default_handler")));
void CAN1_SCE_IRQHandler(void)             __attribute__((weak, alias("default_handler")));
void EXTI9_5_IRQHandler(void)              __attribute__((weak, alias("default_handler")));
void TIM1_BRK_IRQHandler(void)             __attribute__((weak, alias("default_handler")));
void TIM1_UP_IRQHandler(void)              __attribute__((weak, alias("default_handler")));
void TIM1_TRG_COM_IRQHandler(void)         __attribute__((weak, alias("default_handler")));
void TIM1_CC_IRQHandler(void)              __attribute__((weak, alias("default_handler")));
void TIM2_IRQHandler(void)                 __attribute__((weak, alias("default_handler")));
void TIM3_IRQHandler(void)                 __attribute__((weak, alias("default_handler")));
void TIM4_IRQHandler(void)                 __attribute__((weak, alias("default_handler")));
void I2C1_EV_IRQHandler(void)              __attribute__((weak, alias("default_handler")));
void I2C1_ER_IRQHandler(void)              __attribute__((weak, alias("default_handler")));
void I2C2_EV_IRQHandler(void)              __attribute__((weak, alias("default_handler")));
void I2C2_ER_IRQHandler(void)              __attribute__((weak, alias("default_handler")));
void SPI1_IRQHandler(void)                 __attribute__((weak, alias("default_handler")));
void SPI2_IRQHandler(void)                 __attribute__((weak, alias("default_handler")));
void USART1_IRQHandler(void)               __attribute__((weak, alias("default_handler")));
void USART2_IRQHandler(void)               __attribute__((weak, alias("default_handler")));
void USART3_IRQHandler(void)               __attribute__((weak, alias("default_handler")));
void EXTI15_10_IRQHandler(void)            __attribute__((weak, alias("default_handler")));
void RTC_Alarm_IRQHandler(void)            __attribute__((weak, alias("default_handler")));
void USBWakeUp_IRQHandler(void)            __attribute__((weak, alias("default_handler")));
void TIM8_BRK_IRQHandler(void)             __attribute__((weak, alias("default_handler")));
void TIM8_UP_IRQHandler(void)              __attribute__((weak, alias("default_handler")));
void TIM8_TRG_COM_IRQHandler(void)         __attribute__((weak, alias("default_handler")));
void TIM8_CC_IRQHandler(void)              __attribute__((weak, alias("default_handler")));
void ADC3_IRQHandler(void)                 __attribute__((weak, alias("default_handler")));
void FSMC_IRQHandler(void)                 __attribute__((weak, alias("default_handler")));
void SDIO_IRQHandler(void)                 __attribute__((weak, alias("default_handler")));
void TIM5_IRQHandler(void)                 __attribute__((weak, alias("default_handler")));
void SPI3_IRQHandler(void)                 __attribute__((weak, alias("default_handler")));
void UART4_IRQHandler(void)                __attribute__((weak, alias("default_handler")));
void UART5_IRQHandler(void)                __attribute__((weak, alias("default_handler")));
void TIM6_IRQHandler(void)                 __attribute__((weak, alias("default_handler")));
void TIM7_IRQHandler(void)                 __attribute__((weak, alias("default_handler")));
void DMA2_Channel1_IRQHandler(void)        __attribute__((weak, alias("default_handler")));
void DMA2_Channel2_IRQHandler(void)        __attribute__((weak, alias("default_handler")));
void DMA2_Channel3_IRQHandler(void)        __attribute__((weak, alias("default_handler")));
void DMA2_Channel4_5_IRQHandler(void)      __attribute__((weak, alias("default_handler")));

/***************************************************************************************************************
**                                              Interrupt Vector Table
**  Placed in .isr_vector section (linked to the start of FLASH by app.ld).
**  Must match the STM32F103xB vector table order exactly (RM0008 Table 63).
***************************************************************************************************************/
__attribute__((section(".isr_vector")))
uint32_t *_isr_vector[] =
{
    /* ---- Cortex-M3 core exceptions ---- */
    (uint32_t *)&_estack,                   /*  0: Initial Stack Pointer          */
    (uint32_t *)Reset_Handler,              /*  1: Reset                          */
    (uint32_t *)NMI_IRQHandler,             /*  2: NMI                            */
    (uint32_t *)HardFault_IRQHandler,       /*  3: Hard Fault                     */
    (uint32_t *)MemManage_IRQHandler,       /*  4: MPU Fault                      */
    (uint32_t *)BusFault_IRQHandler,        /*  5: Bus Fault                      */
    (uint32_t *)UsageFault_IRQHandler,      /*  6: Usage Fault                    */
    (uint32_t *)0,                          /*  7: Reserved                       */
    (uint32_t *)0,                          /*  8: Reserved                       */
    (uint32_t *)0,                          /*  9: Reserved                       */
    (uint32_t *)0,                          /* 10: Reserved                       */
    (uint32_t *)SVC_IRQHandler,             /* 11: SVCall                         */
    (uint32_t *)DebugMonitor_IRQHandler,    /* 12: Debug Monitor                  */
    (uint32_t *)0,                          /* 13: Reserved                       */
    (uint32_t *)PendSV_IRQHandler,          /* 14: PendSV (context switch)        */
    (uint32_t *)SysTick_IRQHandler,         /* 15: SysTick                        */

    /* ---- External interrupts (IRQ0 … IRQ59) ---- */
    (uint32_t *)WWDG_IRQHandler,            /*  0: Window Watchdog                */
    (uint32_t *)PVD_IRQHandler,             /*  1: PVD through EXTI Line detect   */
    (uint32_t *)TAMPER_IRQHandler,          /*  2: Tamper                         */
    (uint32_t *)RTC_IRQHandler,             /*  3: RTC global                     */
    (uint32_t *)FLASH_IRQHandler,           /*  4: Flash global                   */
    (uint32_t *)RCC_IRQHandler,             /*  5: RCC global                     */
    (uint32_t *)EXTI0_IRQHandler,           /*  6: EXTI Line 0                    */
    (uint32_t *)EXTI1_IRQHandler,           /*  7: EXTI Line 1                    */
    (uint32_t *)EXTI2_IRQHandler,           /*  8: EXTI Line 2                    */
    (uint32_t *)EXTI3_IRQHandler,           /*  9: EXTI Line 3                    */
    (uint32_t *)EXTI4_IRQHandler,           /* 10: EXTI Line 4                    */
    (uint32_t *)DMA1_Channel1_IRQHandler,   /* 11: DMA1 Channel 1                 */
    (uint32_t *)DMA1_Channel2_IRQHandler,   /* 12: DMA1 Channel 2                 */
    (uint32_t *)DMA1_Channel3_IRQHandler,   /* 13: DMA1 Channel 3                 */
    (uint32_t *)DMA1_Channel4_IRQHandler,   /* 14: DMA1 Channel 4                 */
    (uint32_t *)DMA1_Channel5_IRQHandler,   /* 15: DMA1 Channel 5                 */
    (uint32_t *)DMA1_Channel6_IRQHandler,   /* 16: DMA1 Channel 6                 */
    (uint32_t *)DMA1_Channel7_IRQHandler,   /* 17: DMA1 Channel 7                 */
    (uint32_t *)ADC1_2_IRQHandler,          /* 18: ADC1 & ADC2 global             */
    (uint32_t *)USB_HP_CAN1_TX_IRQHandler,  /* 19: USB HP / CAN1 TX               */
    (uint32_t *)USB_LP_CAN1_RX0_IRQHandler, /* 20: USB LP / CAN1 RX0              */
    (uint32_t *)CAN1_RX1_IRQHandler,        /* 21: CAN1 RX1                       */
    (uint32_t *)CAN1_SCE_IRQHandler,        /* 22: CAN1 SCE                       */
    (uint32_t *)EXTI9_5_IRQHandler,         /* 23: EXTI Lines 9-5                 */
    (uint32_t *)TIM1_BRK_IRQHandler,        /* 24: TIM1 Break                     */
    (uint32_t *)TIM1_UP_IRQHandler,         /* 25: TIM1 Update                    */
    (uint32_t *)TIM1_TRG_COM_IRQHandler,    /* 26: TIM1 Trigger/Commutation       */
    (uint32_t *)TIM1_CC_IRQHandler,         /* 27: TIM1 Capture Compare           */
    (uint32_t *)TIM2_IRQHandler,            /* 28: TIM2 global                    */
    (uint32_t *)TIM3_IRQHandler,            /* 29: TIM3 global                    */
    (uint32_t *)TIM4_IRQHandler,            /* 30: TIM4 global                    */
    (uint32_t *)I2C1_EV_IRQHandler,         /* 31: I2C1 Event                     */
    (uint32_t *)I2C1_ER_IRQHandler,         /* 32: I2C1 Error                     */
    (uint32_t *)I2C2_EV_IRQHandler,         /* 33: I2C2 Event                     */
    (uint32_t *)I2C2_ER_IRQHandler,         /* 34: I2C2 Error                     */
    (uint32_t *)SPI1_IRQHandler,            /* 35: SPI1 global                    */
    (uint32_t *)SPI2_IRQHandler,            /* 36: SPI2 global                    */
    (uint32_t *)USART1_IRQHandler,          /* 37: USART1 global                  */
    (uint32_t *)USART2_IRQHandler,          /* 38: USART2 global                  */
    (uint32_t *)USART3_IRQHandler,          /* 39: USART3 global                  */
    (uint32_t *)EXTI15_10_IRQHandler,       /* 40: EXTI Lines 15-10               */
    (uint32_t *)RTC_Alarm_IRQHandler,       /* 41: RTC Alarm through EXTI Line    */
    (uint32_t *)USBWakeUp_IRQHandler,       /* 42: USB WakeUp from suspend        */
    (uint32_t *)TIM8_BRK_IRQHandler,        /* 43: TIM8 Break                     */
    (uint32_t *)TIM8_UP_IRQHandler,         /* 44: TIM8 Update                    */
    (uint32_t *)TIM8_TRG_COM_IRQHandler,    /* 45: TIM8 Trigger/Commutation       */
    (uint32_t *)TIM8_CC_IRQHandler,         /* 46: TIM8 Capture Compare           */
    (uint32_t *)ADC3_IRQHandler,            /* 47: ADC3 global                    */
    (uint32_t *)FSMC_IRQHandler,            /* 48: FSMC global                    */
    (uint32_t *)SDIO_IRQHandler,            /* 49: SDIO global                    */
    (uint32_t *)TIM5_IRQHandler,            /* 50: TIM5 global                    */
    (uint32_t *)SPI3_IRQHandler,            /* 51: SPI3 global                    */
    (uint32_t *)UART4_IRQHandler,           /* 52: UART4 global                   */
    (uint32_t *)UART5_IRQHandler,           /* 53: UART5 global                   */
    (uint32_t *)TIM6_IRQHandler,            /* 54: TIM6 global                    */
    (uint32_t *)TIM7_IRQHandler,            /* 55: TIM7 global                    */
    (uint32_t *)DMA2_Channel1_IRQHandler,   /* 56: DMA2 Channel 1                 */
    (uint32_t *)DMA2_Channel2_IRQHandler,   /* 57: DMA2 Channel 2                 */
    (uint32_t *)DMA2_Channel3_IRQHandler,   /* 58: DMA2 Channel 3                 */
    (uint32_t *)DMA2_Channel4_5_IRQHandler, /* 59: DMA2 Channel 4 & 5            */
};

/***************************************************************************************************************
**                                                End of file
***************************************************************************************************************/
