/*
****************************************************************************************************************
 * @file       memory_map.h
 * @brief      Memory layout and partition definitions for STM32F103xB (128 KB Flash, 20 KB SRAM)
 * @date       2026/10/25
 * @author     [Gentantun] (nguyenthanhtung8196@gmail.com)
 * @details    Defines the flash and SRAM partitioning between the Bootloader, Download (DFU)
 *             staging partition, and Application partition.
 *
 *             Memory partitioning for STM32F103xB (128 KB Flash / 20 KB SRAM):
 *               - 0x08000000 - 0x08007FFF (32 KB) : Bootloader partition
 *               - 0x08008000 - 0x0800FFFF (32 KB) : Download (DWL/DFU) staging area
 *               - 0x08010000 - 0x0801FFFF (64 KB) : Application partition
 *               - 0x20000000 - 0x20004FFF (20 KB) : Internal SRAM
 *
 * @ref        stm32f7/Common/memory_map.h
 * @copyright  Copyright (c) 2026 RoboTun
****************************************************************************************************************
*/
#ifndef _MEMORY_MAP_H_
#define _MEMORY_MAP_H_
#ifdef __cplusplus
extern "C"
{
#endif

/***************************************************************************************************************
**                                          INTERNAL MACRO DEFINITIONS
***************************************************************************************************************/
/* Function pointer invocation helper */
#define FUNC(x)                     (*((void (**)(void))(x)))


/* ---------------------------------------------------------------------------
 * Flash Partitions
 * --------------------------------------------------------------------------- */
/* Bootloader partition (32 KB) */
#define BOOT_START_ADDRESS          0x08000000U
#define BOOT_SIZE                   0x00004000U     /* 16 KB */

/* Download / DFU staging partition (32 KB) */
#define DWL_START_ADDRESS           0x08004000U
#define DWL_SIZE                    0x00004000U     /* 16 KB */

/* Application partition (64 KB) */
#define APP_START_ADDRESS           0x08008000U
#define APP_SIZE                    0x00018000U     /* 96 KB */

/* ---------------------------------------------------------------------------
 * Download entry-point jump helpers
 * --------------------------------------------------------------------------- */
#define entry_dwl                   FUNC(DWL_START_ADDRESS)
#define entry_dwl2                  FUNC((DWL_START_ADDRESS + 4U))

#ifdef __cplusplus
}
#endif
#endif /* _MEMORY_MAP_H_ */
