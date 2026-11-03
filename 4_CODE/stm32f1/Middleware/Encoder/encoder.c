/*
****************************************************************************************************************
 * @file       encoder.c
 * @brief      Incremental quadrature encoder driver for STM32F1 (PB0, PB1)
 * @date       2026/11/03
 * @author     [Gentantun] (nguyenthanhtung8196@gmail.com)
 * @details    Implements 4x quadrature state machine decoding on PB0 (Pin A) and PB1 (Pin B)
 * @copyright  Copyright (c) 2026 RoboTun
****************************************************************************************************************
*/

/***************************************************************************************************************
**                                                   INCLUDES
***************************************************************************************************************/
#include "encoder.h"
#include <string.h>

/***************************************************************************************************************
**                                          INTERNAL MACRO DEFINITIONS
***************************************************************************************************************/
#define TWO_PI                      (6.283185307179586f)

/***************************************************************************************************************
**                                        INTERNAL VARIABLE DEFINITIONS
***************************************************************************************************************/
static encoder_t g_encoder;

/* Quadrature transition lookup table:
 * Index: (previous_state << 2) | current_state
 * Value: +1 (forward), -1 (reverse), 0 (invalid/no move)
 */
static const int8_t QUADRATURE_TABLE[16] = 
{
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

/***************************************************************************************************************
**                                             FUNCTION DEFINITIONS
***************************************************************************************************************/

/**
 * @brief  Initialise GPIO PB0, PB1 and EXTI interrupts for Encoder
 */
void Encoder_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable GPIOB and AFIO clock */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    /* Configure PB0 and PB1 as input with pull-up and interrupt on both edges */
    GPIO_InitStruct.Pin  = ENCODER_PIN_A | ENCODER_PIN_B;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(ENCODER_PORT, &GPIO_InitStruct);

    /* Read initial state */
    uint8_t pin_a = (HAL_GPIO_ReadPin(ENCODER_PORT, ENCODER_PIN_A) == GPIO_PIN_SET) ? 1U : 0U;
    uint8_t pin_b = (HAL_GPIO_ReadPin(ENCODER_PORT, ENCODER_PIN_B) == GPIO_PIN_SET) ? 1U : 0U;

    memset(&g_encoder, 0, sizeof(encoder_t));
    g_encoder.last_state = (pin_a << 1U) | pin_b;

    /* Configure NVIC for EXTI Line 0 (PB0) and Line 1 (PB1) */
    HAL_NVIC_SetPriority(ENCODER_EXTI_IRQN_A, 3, 0);
    HAL_NVIC_EnableIRQ(ENCODER_EXTI_IRQN_A);

    HAL_NVIC_SetPriority(ENCODER_EXTI_IRQN_B, 3, 0);
    HAL_NVIC_EnableIRQ(ENCODER_EXTI_IRQN_B);
}

/**
 * @brief  Reset encoder position counters
 */
void Encoder_Reset(void)
{
    g_encoder.total_count = 0;
    g_encoder.speed_count = 0;
    g_encoder.speed_rpm   = 0.0f;
    g_encoder.speed_rad_s = 0.0f;
    g_encoder.total_revolutions = 0.0f;
    g_encoder.angle_deg   = 0.0f;
}

/**
 * @brief  Get current raw pulse count
 * @return Total signed pulse count
 */
int64_t Encoder_GetCount(void)
{
    return g_encoder.total_count;
}

/**
 * @brief  Get current wheel angle in degrees
 * @return Wheel angle in degrees
 */
float Encoder_GetAngleDeg(void)
{
    return g_encoder.angle_deg;
}

/**
 * @brief  Get current wheel angle in radians
 * @return Wheel angle in radians
 */
float Encoder_GetAngleRad(void)
{
    return g_encoder.total_revolutions * TWO_PI;
}

/**
 * @brief  Calculate speed and position. Call periodically (e.g. in 200Hz or 100Hz loop)
 * @param  dt: Time elapsed since last update in seconds (e.g. 0.005f for 200Hz)
 */
void Encoder_Update(float dt)
{
    if (dt <= 0.0f)
    {
        return;
    }

    /* Atomically read and reset delta pulse count */
    __disable_irq();
    int32_t delta_ticks = g_encoder.speed_count;
    g_encoder.speed_count = 0;
    int64_t total_ticks = g_encoder.total_count;
    __enable_irq();

    /* Calculate RPM:
     * Revolutions in dt = delta_ticks / ENCODER_COUNTS_PER_REV
     * Rev/sec = (delta_ticks / ENCODER_COUNTS_PER_REV) / dt
     * RPM = Rev/sec * 60.0f
     */
    float revs_per_sec = ((float)delta_ticks / ENCODER_COUNTS_PER_REV) / dt;
    g_encoder.speed_rpm   = revs_per_sec * 60.0f;
    g_encoder.speed_rad_s = revs_per_sec * TWO_PI;

    /* Total full revolutions and accumulated angle in degrees */
    g_encoder.total_revolutions = (float)total_ticks / ENCODER_COUNTS_PER_REV;
    g_encoder.angle_deg         = g_encoder.total_revolutions * 360.0f;
}

/**
 * @brief  Get calculated speed in RPM
 * @return Speed in RPM
 */
float Encoder_GetRPM(void)
{
    return g_encoder.speed_rpm;
}

/**
 * @brief  Get calculated angular velocity in rad/s
 * @return Angular velocity in rad/s
 */
float Encoder_GetRadPerSec(void)
{
    return g_encoder.speed_rad_s;
}

/**
 * @brief  Encoder EXTI interrupt callback (called from EXTI0 & EXTI1 handlers)
 */
void Encoder_EXTI_Callback(void)
{
    /* Read current levels of PB0 and PB1 directly */
    uint8_t pin_a = (HAL_GPIO_ReadPin(ENCODER_PORT, ENCODER_PIN_A) == GPIO_PIN_SET) ? 1U : 0U;
    uint8_t pin_b = (HAL_GPIO_ReadPin(ENCODER_PORT, ENCODER_PIN_B) == GPIO_PIN_SET) ? 1U : 0U;
    uint8_t current_state = (pin_a << 1U) | pin_b;

    /* Decode direction from state transition */
    uint8_t index = (g_encoder.last_state << 2U) | current_state;
    int8_t step = QUADRATURE_TABLE[index];

    if (step != 0)
    {
        g_encoder.total_count += step;
        g_encoder.speed_count += step;
    }

    g_encoder.last_state = current_state;
}

/***************************************************************************************************************
**                                            EXTI INTERRUPT HANDLERS
***************************************************************************************************************/

/**
 * @brief  EXTI0 IRQ Handler (PB0 / Channel A)
 */
void EXTI0_IRQHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(ENCODER_PIN_A) != RESET)
    {
        __HAL_GPIO_EXTI_CLEAR_IT(ENCODER_PIN_A);
        Encoder_EXTI_Callback();
    }
}

/**
 * @brief  EXTI1 IRQ Handler (PB1 / Channel B)
 */
void EXTI1_IRQHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(ENCODER_PIN_B) != RESET)
    {
        __HAL_GPIO_EXTI_CLEAR_IT(ENCODER_PIN_B);
        Encoder_EXTI_Callback();
    }
}

