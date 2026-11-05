/*
****************************************************************************************************************
 * @file       encoder.h
 * @brief      Dual incremental quadrature encoder driver for STM32F1
 *             - Left Encoder:  PB0 (ChA / EXTI0),  PB1 (ChB / EXTI1)
 *             - Right Encoder: PB10 (ChA / EXTI10), PB11 (ChB / EXTI11)
 * @date       2026/11/05
 * @author     [Gentantun] (nguyenthanhtung8196@gmail.com)
 * @details    Reads 2-phase quadrature encoders for both Left and Right wheels
 *             using GPIO EXTI interrupts with 4x state-machine decoding.
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
/* Left Encoder hardware mapping */
#define ENCODER_LEFT_PORT               GPIOB
#define ENCODER_LEFT_PIN_A              GPIO_PIN_0
#define ENCODER_LEFT_PIN_B              GPIO_PIN_1
#define ENCODER_LEFT_EXTI_IRQN_A        EXTI0_IRQn
#define ENCODER_LEFT_EXTI_IRQN_B        EXTI1_IRQn

/* Right Encoder hardware mapping */
#define ENCODER_RIGHT_PORT              GPIOB
#define ENCODER_RIGHT_PIN_A             GPIO_PIN_10
#define ENCODER_RIGHT_PIN_B             GPIO_PIN_11
#define ENCODER_RIGHT_EXTI_IRQN         EXTI15_10_IRQn

/* Resolution configuration:
 * Measured empirically on the wheel output shaft: 2550 counts per revolution (4x decoding)
 */
#define ENCODER_COUNTS_PER_REV          (2550.0f)

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
 * @brief  Initialise both Left (PB0, PB1) and Right (PB10, PB11) Encoders
 */
void Encoder_Init(void);

/**
 * @brief  Initialise Left Encoder GPIO and EXTI interrupts (PB0, PB1)
 */
void Encoder_Left_Init(void);

/**
 * @brief  Initialise Right Encoder GPIO and EXTI interrupts (PB10, PB11)
 */
void Encoder_Right_Init(void);

/**
 * @brief  Reset both encoder position counters
 */
void Encoder_Reset(void);

/**
 * @brief  Reset Left encoder position counters
 */
void Encoder_Left_Reset(void);

/**
 * @brief  Reset Right encoder position counters
 */
void Encoder_Right_Reset(void);

/**
 * @brief  Calculate speed and position for both encoders (call periodically, e.g. 200Hz)
 * @param  dt: Time elapsed since last update in seconds
 */
void Encoder_Update(float dt);

/**
 * @brief  Calculate speed and position for Left encoder
 * @param  dt: Time elapsed in seconds
 */
void Encoder_Left_Update(float dt);

/**
 * @brief  Calculate speed and position for Right encoder
 * @param  dt: Time elapsed in seconds
 */
void Encoder_Right_Update(float dt);

/**
 * @brief  Get Left encoder raw pulse count
 * @return Signed pulse count
 */
int64_t Encoder_Left_GetCount(void);

/**
 * @brief  Get Right encoder raw pulse count
 * @return Signed pulse count
 */
int64_t Encoder_Right_GetCount(void);

/**
 * @brief  Get Left wheel angle in degrees
 * @return Wheel angle in degrees
 */
float Encoder_Left_GetAngleDeg(void);

/**
 * @brief  Get Right wheel angle in degrees
 * @return Wheel angle in degrees
 */
float Encoder_Right_GetAngleDeg(void);

/**
 * @brief  Get Left wheel angle in radians
 * @return Wheel angle in radians
 */
float Encoder_Left_GetAngleRad(void);

/**
 * @brief  Get Right wheel angle in radians
 * @return Wheel angle in radians
 */
float Encoder_Right_GetAngleRad(void);

/**
 * @brief  Get Left wheel speed in RPM
 * @return Speed in RPM
 */
float Encoder_Left_GetRPM(void);

/**
 * @brief  Get Right wheel speed in RPM
 * @return Speed in RPM
 */
float Encoder_Right_GetRPM(void);

/**
 * @brief  Get Left wheel angular velocity in rad/s
 * @return Angular velocity in rad/s
 */
float Encoder_Left_GetRadPerSec(void);

/**
 * @brief  Get Right wheel angular velocity in rad/s
 * @return Angular velocity in rad/s
 */
float Encoder_Right_GetRadPerSec(void);

/**
 * @brief  Left encoder EXTI callback
 */
void Encoder_Left_EXTI_Callback(void);

/**
 * @brief  Right encoder EXTI callback
 */
void Encoder_Right_EXTI_Callback(void);

#ifdef __cplusplus
}
#endif

#endif /* _ENCODER_H_ */
