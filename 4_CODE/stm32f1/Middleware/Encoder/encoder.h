/*
****************************************************************************************************************
 * @file       encoder.h
 * @brief      Incremental quadrature encoder driver for STM32F1 (PB0, PB1)
 * @date       2026/11/03
 * @author     [Gentantun] (nguyenthanhtung8196@gmail.com)
 * @details    Reads 2-phase quadrature encoder on PB0 (Channel A) and PB1 (Channel B)
 *             using GPIO EXTI interrupts with 4x resolution (44 counts/rev for 11 PPR).
 * @copyright  Copyright (c) 2026 RoboTun
****************************************************************************************************************
*/
#ifndef _ENCODER_H_
#define _ENCODER_H_

#ifdef __cplusplus
extern "C"
{
#endif

/***************************************************************************************************************
**                                                   INCLUDES
***************************************************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "stm32f1xx_hal.h"

/***************************************************************************************************************
**                                            MACROS & CONSTANTS
***************************************************************************************************************/
/* Encoder hardware mapping */
#define ENCODER_PORT                GPIOB
#define ENCODER_PIN_A               GPIO_PIN_0
#define ENCODER_PIN_B               GPIO_PIN_1

#define ENCODER_EXTI_IRQN_A         EXTI0_IRQn
#define ENCODER_EXTI_IRQN_B         EXTI1_IRQn

/* Resolution configuration:
 * Measured empirically on the wheel output shaft: 2552 counts per revolution (4x decoding)
 * (Corresponds to ~0.141 degrees per tick resolution)
 */
#define ENCODER_COUNTS_PER_REV      (2500.0f)

/***************************************************************************************************************
**                                            TYPE DEFINITIONS
***************************************************************************************************************/
typedef struct
{
    volatile int64_t total_count;       /* Total accumulated pulse count (signed) */
    volatile int32_t speed_count;       /* Pulse count delta in sample period dt */
    
    float    speed_rpm;                 /* Calculated speed in Revolutions Per Minute */
    float    speed_rad_s;               /* Calculated angular velocity in rad/s */
    float    total_revolutions;         /* Total number of full wheel revolutions */
    float    angle_deg;                 /* Total accumulated wheel angle in degrees */
    
    uint8_t  last_state;                /* Previous 2-bit state of (Channel_A << 1 | Channel_B) */
} encoder_t;

/***************************************************************************************************************
**                                         FUNCTION PROTOTYPES
***************************************************************************************************************/

/**
 * @brief  Initialise GPIO PB0, PB1 and EXTI interrupts for Encoder
 */
void Encoder_Init(void);

/**
 * @brief  Reset encoder position counters
 */
void Encoder_Reset(void);

/**
 * @brief  Get current raw pulse count
 * @return Total signed pulse count
 */
int64_t Encoder_GetCount(void);

/**
 * @brief  Get current wheel angle in degrees
 * @return Wheel angle in degrees
 */
float Encoder_GetAngleDeg(void);

/**
 * @brief  Get current wheel angle in radians
 * @return Wheel angle in radians
 */
float Encoder_GetAngleRad(void);

/**
 * @brief  Calculate speed and position. Call periodically (e.g. in 200Hz or 100Hz loop)
 * @param  dt: Time elapsed since last update in seconds (e.g. 0.005f for 200Hz)
 */
void Encoder_Update(float dt);

/**
 * @brief  Get calculated speed in RPM
 * @return Speed in RPM
 */
float Encoder_GetRPM(void);

/**
 * @brief  Get calculated angular velocity in rad/s
 * @return Angular velocity in rad/s
 */
float Encoder_GetRadPerSec(void);

/**
 * @brief  Encoder EXTI interrupt callback (called from EXTI0 & EXTI1 handlers)
 */
void Encoder_EXTI_Callback(void);

#ifdef __cplusplus
}
#endif

#endif /* _ENCODER_H_ */

