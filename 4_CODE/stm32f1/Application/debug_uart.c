/*
****************************************************************************************************************
 * @file       debug_uart.c
 * @brief      Lightweight debug UART implementation for STM32F103xB (direct register access)
 * @date       2026/10/25
 * @author     [Gentantun] (nguyenthanhtung8196@gmail.com)
 * @details    Configures USART1 (PA9, TX-only) at 115200 baud.
 *             No HAL dependency — purely register-level to keep the
 *             bootloader footprint small and avoid HAL initialisation order issues.
 *
 *             Key differences vs STM32F7 version:
 *               - STM32F1 GPIO uses CRL/CRH registers (no MODER/AFR).
 *               - STM32F1 USART uses SR + DR instead of ISR + TDR.
 *               - APB2 clock = 72 MHz  →  BRR = 625 (0x271) for 115200.
 *
 * @ref        stm32f7/Application/debug_uart.c (original template)
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
/* BRR value for 115200 baud @ APB2 = 72 MHz
 * BRR = fPCLK / baud = 72 000 000 / 115 200 = 625 = 0x271              */
#define DEBUG_UART_BRR_VALUE    625U

/***************************************************************************************************************
**                                             FUNCTION DEFINITIONS
***************************************************************************************************************/

/**
 * @brief  Initialise USART1 for debug TX output (PA9, 115200 8N1).
 *
 *         Step 1 : Enable peripheral clocks (GPIOA on APB2, USART1 on APB2).
 *         Step 2 : Configure PA9 as Alternate Function Push-Pull, 50 MHz.
 *                  On STM32F1 PA9 is in CRH (pins 8-15).
 *                  CNF[1:0] = 10 (AF push-pull), MODE[1:0] = 11 (50 MHz).
 *                  Bit field for pin 9 in CRH: bits [7:4].
 *         Step 3 : Configure USART1 registers.
 */
void debug_uart_init(void)
{
    /* ------------------------------------------------------------------ */
    /* Step 1: Enable clocks                                               */
    /* ------------------------------------------------------------------ */

    /* GPIOA clock on APB2 */
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

    /* USART1 clock on APB2 */
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    /* Small read-back delay to let clock propagate */
    volatile uint32_t tmpreg = RCC->APB2ENR;
    (void)tmpreg;

    /* ------------------------------------------------------------------ */
    /* Step 2: Configure PA9 — Alternate Function Push-Pull 50 MHz        */
    /* CRH controls pins 8-15.  Pin 9 occupies bits [7:4].                */
    /* CNF = 10b (AF push-pull), MODE = 11b (output 50 MHz)               */
    /* ------------------------------------------------------------------ */
    GPIOA->CRH &= ~(0xFU << 4U);                   /* Clear bits [7:4] for pin 9 */
    GPIOA->CRH |=  (0xBU << 4U);                   /* MODE=11b, CNF=10b -> 0xB   */

    /* ------------------------------------------------------------------ */
    /* Step 3: Configure USART1                                            */
    /* ------------------------------------------------------------------ */

    /* Disable USART before configuration */
    USART1->CR1 = 0U;

    /* Baud rate: APB2 (72 MHz) / 115200 ≈ 625 */
    USART1->BRR = DEBUG_UART_BRR_VALUE;

    /* CR2: 1 stop bit (default = 0) */
    USART1->CR2 = 0U;

    /* CR3: no flow control (default = 0) */
    USART1->CR3 = 0U;

    /* Enable Transmitter + USART (TE=bit3, UE=bit13) */
    USART1->CR1 = USART_CR1_TE   /* Transmitter enable */
                | USART_CR1_UE;  /* USART enable       */
}

/* ------------------------------------------------------------------ */

/**
 * @brief  Transmit a single character over USART1 (blocking).
 *         Waits until the TX data register is empty (TXE flag in SR).
 *         On STM32F1 the status register is SR and data register is DR.
 * @param  c  Character to transmit.
 */
void debug_uart_putchar(char c)
{
    /* Wait until TX data register empty (TXE = bit 7 of SR) */
    while (!(USART1->SR & USART_SR_TXE))
    {
        /* busy wait */
    }
    USART1->DR = (uint8_t)c;
}

/***************************************************************************************************************
**                                                End of file
***************************************************************************************************************/
