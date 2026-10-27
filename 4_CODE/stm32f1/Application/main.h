/*
****************************************************************************************************************
 * @file       main.h
 * @brief      Main application header for STM32F1xx (Cortex-M3) with ThreadX RTOS
 * @date       2026/10/25
 * @author     [Gentantun] (nguyenthanhtung8196@gmail.com)
 * @details    Includes HAL, CMSIS and project-common headers needed by main.c
 * @ref        stm32f7/Application/main.h (original template)
 * @copyright  Copyright (c) 2026 RoboTun
****************************************************************************************************************
*/
#ifndef _MAIN_H_
#define _MAIN_H_
#ifdef __cplusplus
extern "C"
{
#endif

/***************************************************************************************************************
**                                                   INCLUDES
***************************************************************************************************************/
#include <stdint.h>
#include <stdio.h>
#include "common.h"
#include "memory_map.h"
#include "stm32f1xx_hal_conf.h"
#include "stm32f1xx_hal.h"

/***************************************************************************************************************
**                                            FUNCTION PROTOTYPES
***************************************************************************************************************/
void led_init(void);
void toggle_led(void);

#ifdef __cplusplus
}
#endif
#endif /* _MAIN_H_ */
