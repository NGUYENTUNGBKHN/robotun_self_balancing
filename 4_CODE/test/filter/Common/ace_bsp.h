/*
****************************************************************************************************************
 * @file       ace_bsp.h
 * @brief      Board Support Package (BSP) pin definitions and hardware mappings for STM32F103xB
 * @date       2026/10/25
 * @author     [Gentantun] (nguyenthanhtung8196@gmail.com)
 * @details    Maps all hardware peripheral pins, status LEDs, debug UART, and motor control
 *             signals for the STM32F103xB self-balancing robot controller.
 * @ref        stm32f7/Common/ace_bsp.h
 * @copyright  Copyright (c) 2026 RoboTun
****************************************************************************************************************
*/
#ifndef _ACE_BSP_H_
#define _ACE_BSP_H_
#ifdef __cplusplus
extern "C"
{
#endif

/***************************************************************************************************************
**                                                   INCLUDES
***************************************************************************************************************/
#include "stm32f1xx_hal.h"

/***************************************************************************************************************
**                                          INTERNAL MACRO DEFINITIONS
***************************************************************************************************************/
/* General GPIO Pin Indices */
#define MCU_GPIO_PIN_0                  0U
#define MCU_GPIO_PIN_1                  1U
#define MCU_GPIO_PIN_2                  2U
#define MCU_GPIO_PIN_3                  3U
#define MCU_GPIO_PIN_4                  4U
#define MCU_GPIO_PIN_5                  5U
#define MCU_GPIO_PIN_6                  6U
#define MCU_GPIO_PIN_7                  7U
#define MCU_GPIO_PIN_8                  8U
#define MCU_GPIO_PIN_9                  9U
#define MCU_GPIO_PIN_10                 10U
#define MCU_GPIO_PIN_11                 11U
#define MCU_GPIO_PIN_12                 12U
#define MCU_GPIO_PIN_13                 13U
#define MCU_GPIO_PIN_14                 14U
#define MCU_GPIO_PIN_15                 15U

/* ---------------------------------------------------------------------------
 * System Clock Configuration Defaults
 * --------------------------------------------------------------------------- */
#define BSP_SYSCLK_FREQ_HZ              72000000U   /* 72 MHz SYSCLK */
#define BSP_HCLK_FREQ_HZ                72000000U   /* 72 MHz AHB */
#define BSP_PCLK1_FREQ_HZ               36000000U   /* 36 MHz APB1 (max 36 MHz) */
#define BSP_PCLK2_FREQ_HZ               72000000U   /* 72 MHz APB2 */

/* ---------------------------------------------------------------------------
 * User / Status LED (PC13, active-low on standard Blue-Pill board)
 * --------------------------------------------------------------------------- */
#define LED_PIN                         GPIO_PIN_13
#define LED_PORT                        GPIOC
#define LED_CLK_ENABLE()                __HAL_RCC_GPIOC_CLK_ENABLE()
#define LED_CLK_DISABLE()               __HAL_RCC_GPIOC_CLK_DISABLE()

/* ---------------------------------------------------------------------------
 * Debug UART Interface (USART1: TX on PA9, RX on PA10, 115200 baud)
 * --------------------------------------------------------------------------- */
#define DEBUG_UART_INSTANCE             USART1
#define DEBUG_UART_BAUDRATE             115200U
#define DEBUG_UART_TX_PIN               GPIO_PIN_9
#define DEBUG_UART_TX_PORT              GPIOA
#define DEBUG_UART_RX_PIN               GPIO_PIN_10
#define DEBUG_UART_RX_PORT              GPIOA
#define DEBUG_UART_CLK_ENABLE()         do { \
                                            __HAL_RCC_GPIOA_CLK_ENABLE(); \
                                            __HAL_RCC_USART1_CLK_ENABLE(); \
                                        } while (0)

/* ---------------------------------------------------------------------------
 * Stepper Motor Driver Hardware Mapping (TIM3 CH1 on PA6, DIR on PB5)
 * --------------------------------------------------------------------------- */
#define STEPPER_STEP_PIN                GPIO_PIN_6
#define STEPPER_STEP_PORT               GPIOA
#define STEPPER_DIR_PIN                 GPIO_PIN_5
#define STEPPER_DIR_PORT                GPIOB
#define STEPPER_TIM_INSTANCE            TIM3
#define STEPPER_TIM_CHANNEL             TIM_CHANNEL_1
#define STEPPER_TIM_IRQn                TIM3_IRQn
#define STEPPER_CLK_ENABLE()            do { \
                                            __HAL_RCC_GPIOA_CLK_ENABLE(); \
                                            __HAL_RCC_GPIOB_CLK_ENABLE(); \
                                            __HAL_RCC_TIM3_CLK_ENABLE(); \
                                        } while (0)

#ifdef __cplusplus
}
#endif
#endif /* _ACE_BSP_H_ */
