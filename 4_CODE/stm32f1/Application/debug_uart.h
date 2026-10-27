/*
****************************************************************************************************************
 * @file       debug_uart.h
 * @brief      Lightweight debug UART for STM32F103xB (no HAL dependency)
 * @date       2026/10/25
 * @author     [Gentantun] (nguyenthanhtung8196@gmail.com)
 * @details    Configures USART1 on PA9 (TX-only, 115200 8N1) using direct
 *             register access.  printf() is redirected to this UART via
 *             the _write() newlib syscall in syscalls.c.
 *
 *             Hardware mapping (STM32F103 Blue-Pill / Maple Mini):
 *               USART1_TX  →  PA9  (remap off = default AF)
 *
 *             Clock assumptions (set by SystemClock_Config in main.c):
 *               SYSCLK = 72 MHz
 *               APB2   = SYSCLK / 1 = 72 MHz  →  USART1 clock = 72 MHz
 *
 *             BRR for 115200 baud:
 *               BRR = 72 000 000 / 115200 ≈ 625 (0x271)
 *
 * @ref        stm32f7/Application/debug_uart.h
 * @copyright  Copyright (c) 2026 RoboTun
****************************************************************************************************************
*/
#ifndef DEBUG_UART_H_
#define DEBUG_UART_H_

#include "../SDK/Device/Inc/stm32f1xx.h"

/**
 * @brief  Initialise USART1 for debug TX output.
 *         PA9  →  USART1_TX (default AF, no remap needed on STM32F1)
 *         Baud : 115200 @ 72 MHz APB2
 *         Frame: 8 data bits, no parity, 1 stop bit
 *         TX only, polling mode.
 */
void debug_uart_init(void);

/**
 * @brief  Transmit a single character (blocking poll on TXE).
 * @param  c  Character to send.
 */
void debug_uart_putchar(char c);

#endif /* DEBUG_UART_H_ */
