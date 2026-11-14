/*
****************************************************************************************************************
 * @file       debug_uart.c
 * @brief      Direct-register debug UART driver for STM32F1 Bootloader (USART1 on PA9 @ 115200)
 * @date       2026/10/25
 * @author     [Gentantun] (nguyenthanhtung8196@gmail.com)
 * @details    Configures USART1 TX on PA9 without HAL overhead for minimal footprint.
 *             APB2 = 72 MHz -> BRR = 72000000 / 115200 = 625 (0x271).
 * @ref        stm32f7/Bootloader/debug_uart.c
 * @copyright  Copyright (c) 2026 RoboTun
****************************************************************************************************************
*/
/***************************************************************************************************************
**                                                   INCLUDES
***************************************************************************************************************/
#include "debug_uart.h"

/***************************************************************************************************************
**                                          INTERNAL MACRO DEFINITIONS
***************************************************************************************************************/
#define DEBUG_UART_BRR_VALUE    625U   /* 72 MHz / 115200 baud */

/***************************************************************************************************************
**                                             FUNCTION DEFINITIONS
***************************************************************************************************************/

/**
 * @brief  Initialise USART1 peripheral and PA9 pin as AF Push-Pull.
 */
void debug_uart_init(void)
{
    /* Enable GPIOA and USART1 peripheral clocks on APB2 */
    RCC->APB2ENR |= (RCC_APB2ENR_IOPAEN | RCC_APB2ENR_USART1EN);

    /* Small read-back delay */
    volatile uint32_t tmpreg = RCC->APB2ENR;
    (void)tmpreg;

    /* Configure PA9 (CRH bits [7:4]): Alternate Function Push-Pull 50 MHz (MODE=11b, CNF=10b -> 0x0B) */
    GPIOA->CRH &= ~(0xFU << 4U);
    GPIOA->CRH |=  (0xBU << 4U);

    /* Disable USART before setup */
    USART1->CR1 = 0U;

    /* Set baud rate divisor */
    USART1->BRR = DEBUG_UART_BRR_VALUE;

    /* 1 stop bit, no flow control */
    USART1->CR2 = 0U;
    USART1->CR3 = 0U;

    /* Enable transmitter and USART module */
    USART1->CR1 = (USART_CR1_TE | USART_CR1_UE);
}

/**
 * @brief  Transmit a single character over USART1 (blocking wait on TXE).
 * @param  c  Character byte.
 */
void debug_uart_putchar(char c)
{
    while (!(USART1->SR & USART_SR_TXE))
    {
    }
    USART1->DR = (uint8_t)c;
}

/***************************************************************************************************************
**                                                End of file
***************************************************************************************************************/
