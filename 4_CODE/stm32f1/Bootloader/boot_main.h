/*
****************************************************************************************************************
 * @file       boot_main.h
 * @brief      Bootloader main header for STM32F103xB
 * @date       2026/10/25
 * @author     [Gentantun] (nguyenthanhtung8196@gmail.com)
 * @details    Declares bootloader system types, HAL inclusions, and bootloader interfaces.
 * @ref        stm32f7/Bootloader/boot_main.h
 * @copyright  Copyright (c) 2026 RoboTun
****************************************************************************************************************
*/
#ifndef _BOOT_MAIN_H_
#define _BOOT_MAIN_H_
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
#include "bootloader.h"
#include "stm32f1xx_hal.h"

/***************************************************************************************************************
**                                         EXTERNAL FUNCTION PROTOTYPES
***************************************************************************************************************/
int boot_main(void);

#ifdef __cplusplus
}
#endif
#endif /* _BOOT_MAIN_H_ */
