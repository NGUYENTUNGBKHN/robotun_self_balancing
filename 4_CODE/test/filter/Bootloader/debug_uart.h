/*
****************************************************************************************************************
 * @file       debug_uart.h
 * @brief      Debug UART driver interface for STM32F1 Bootloader
 * @date       2026/10/25
 * @author     [Gentantun] (nguyenthanhtung8196@gmail.com)
 * @details    Declares direct-register UART initialization and transmission functions for USART1 (PA9).
 * @ref        stm32f7/Bootloader/debug_uart.h
 * @copyright  Copyright (c) 2026 RoboTun
****************************************************************************************************************
*/
#ifndef _DEBUG_UART_H_
#define _DEBUG_UART_H_
#ifdef __cplusplus
extern "C"
{
#endif

/***************************************************************************************************************
**                                                   INCLUDES
***************************************************************************************************************/
#include <stdint.h>
#include "stm32f1xx.h"

/***************************************************************************************************************
**                                         EXTERNAL FUNCTION PROTOTYPES
***************************************************************************************************************/
void debug_uart_init(void);
void debug_uart_putchar(char c);

#ifdef __cplusplus
}
#endif
#endif /* _DEBUG_UART_H_ */
