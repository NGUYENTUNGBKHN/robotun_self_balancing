/*
****************************************************************************************************************
 * @file       encoder.c
 * @brief      Dual incremental quadrature encoder driver for STM32F1
 *             - Left:  PB0 (ChA / EXTI0),  PB1 (ChB / EXTI1)
 *             - Right: PB10 (ChA / EXTI10), PB11 (ChB / EXTI11)
 * @date       2026/11/05
 * @author     [Gentantun] (nguyenthanhtung8196@gmail.com)
 * @details    Implements 4x quadrature state machine decoding for Left and Right encoders
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
static encoder_t g_encoder_left;
static encoder_t g_encoder_right;

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
 * @brief  Initialise Left Encoder (PB0, PB1)
 */
void Encoder_Left_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    GPIO_InitStruct.Pin  = ENCODER_LEFT_PIN_A | ENCODER_LEFT_PIN_B;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(ENCODER_LEFT_PORT, &GPIO_InitStruct);

    uint8_t pin_a = (HAL_GPIO_ReadPin(ENCODER_LEFT_PORT, ENCODER_LEFT_PIN_A) == GPIO_PIN_SET) ? 1U : 0U;
    uint8_t pin_b = (HAL_GPIO_ReadPin(ENCODER_LEFT_PORT, ENCODER_LEFT_PIN_B) == GPIO_PIN_SET) ? 1U : 0U;

    memset(&g_encoder_left, 0, sizeof(encoder_t));
    g_encoder_left.last_state = (pin_a << 1U) | pin_b;

    HAL_NVIC_SetPriority(ENCODER_LEFT_EXTI_IRQN_A, 3, 0);
    HAL_NVIC_EnableIRQ(ENCODER_LEFT_EXTI_IRQN_A);

    HAL_NVIC_SetPriority(ENCODER_LEFT_EXTI_IRQN_B, 3, 0);
    HAL_NVIC_EnableIRQ(ENCODER_LEFT_EXTI_IRQN_B);
}

/**
 * @brief  Initialise Right Encoder (PB10, PB11)
 */
void Encoder_Right_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    GPIO_InitStruct.Pin  = ENCODER_RIGHT_PIN_A | ENCODER_RIGHT_PIN_B;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(ENCODER_RIGHT_PORT, &GPIO_InitStruct);

    uint8_t pin_a = (HAL_GPIO_ReadPin(ENCODER_RIGHT_PORT, ENCODER_RIGHT_PIN_A) == GPIO_PIN_SET) ? 1U : 0U;
    uint8_t pin_b = (HAL_GPIO_ReadPin(ENCODER_RIGHT_PORT, ENCODER_RIGHT_PIN_B) == GPIO_PIN_SET) ? 1U : 0U;

    memset(&g_encoder_right, 0, sizeof(encoder_t));
    g_encoder_right.last_state = (pin_a << 1U) | pin_b;

    HAL_NVIC_SetPriority(ENCODER_RIGHT_EXTI_IRQN, 3, 0);
    HAL_NVIC_EnableIRQ(ENCODER_RIGHT_EXTI_IRQN);
}

/**
 * @brief  Initialise both Encoders
 */
void Encoder_Init(void)
{
    Encoder_Left_Init();
    Encoder_Right_Init();
}

/**
 * @brief  Reset Left encoder position counters
 */
void Encoder_Left_Reset(void)
{
    g_encoder_left.total_count = 0;
    g_encoder_left.speed_count = 0;
    g_encoder_left.speed_rpm   = 0.0f;
    g_encoder_left.speed_rad_s = 0.0f;
    g_encoder_left.total_revolutions = 0.0f;
    g_encoder_left.angle_deg   = 0.0f;
}

/**
 * @brief  Reset Right encoder position counters
 */
void Encoder_Right_Reset(void)
{
    g_encoder_right.total_count = 0;
    g_encoder_right.speed_count = 0;
    g_encoder_right.speed_rpm   = 0.0f;
    g_encoder_right.speed_rad_s = 0.0f;
    g_encoder_right.total_revolutions = 0.0f;
    g_encoder_right.angle_deg   = 0.0f;
}

/**
 * @brief  Reset both encoder position counters
 */
void Encoder_Reset(void)
{
    Encoder_Left_Reset();
    Encoder_Right_Reset();
}

/**
 * @brief  Calculate speed and position for Left encoder
 * @param  dt: Time elapsed in seconds
 */
void Encoder_Left_Update(float dt)
{
    if (dt <= 0.0f)
    {
        return;
    }

    __disable_irq();
    int32_t delta_ticks = g_encoder_left.speed_count;
    g_encoder_left.speed_count = 0;
    int64_t total_ticks = g_encoder_left.total_count;
    __enable_irq();

    float revs_per_sec = ((float)delta_ticks / ENCODER_COUNTS_PER_REV) / dt;
    g_encoder_left.speed_rpm   = revs_per_sec * 60.0f;
    g_encoder_left.speed_rad_s = revs_per_sec * TWO_PI;

    g_encoder_left.total_revolutions = (float)total_ticks / ENCODER_COUNTS_PER_REV;
    g_encoder_left.angle_deg         = g_encoder_left.total_revolutions * 360.0f;
}

/**
 * @brief  Calculate speed and position for Right encoder
 * @param  dt: Time elapsed in seconds
 */
void Encoder_Right_Update(float dt)
{
    if (dt <= 0.0f)
    {
        return;
    }

    __disable_irq();
    int32_t delta_ticks = g_encoder_right.speed_count;
    g_encoder_right.speed_count = 0;
    int64_t total_ticks = g_encoder_right.total_count;
    __enable_irq();

    float revs_per_sec = ((float)delta_ticks / ENCODER_COUNTS_PER_REV) / dt;
    g_encoder_right.speed_rpm   = revs_per_sec * 60.0f;
    g_encoder_right.speed_rad_s = revs_per_sec * TWO_PI;

    g_encoder_right.total_revolutions = (float)total_ticks / ENCODER_COUNTS_PER_REV;
    g_encoder_right.angle_deg         = g_encoder_right.total_revolutions * 360.0f;
}

/**
 * @brief  Calculate speed and position for both encoders
 * @param  dt: Time elapsed in seconds
 */
void Encoder_Update(float dt)
{
    Encoder_Left_Update(dt);
    Encoder_Right_Update(dt);
}

/**
 * @brief  Get Left encoder raw pulse count
 * @return Signed pulse count
 */
int64_t Encoder_Left_GetCount(void)
{
    return g_encoder_left.total_count;
}

/**
 * @brief  Get Right encoder raw pulse count
 * @return Signed pulse count
 */
int64_t Encoder_Right_GetCount(void)
{
    return g_encoder_right.total_count;
}

/**
 * @brief  Get Left wheel angle in degrees
 * @return Wheel angle in degrees
 */
float Encoder_Left_GetAngleDeg(void)
{
    return g_encoder_left.angle_deg;
}

/**
 * @brief  Get Right wheel angle in degrees
 * @return Wheel angle in degrees
 */
float Encoder_Right_GetAngleDeg(void)
{
    return g_encoder_right.angle_deg;
}

/**
 * @brief  Get Left wheel angle in radians
 * @return Wheel angle in radians
 */
float Encoder_Left_GetAngleRad(void)
{
    return g_encoder_left.total_revolutions * TWO_PI;
}

/**
 * @brief  Get Right wheel angle in radians
 * @return Wheel angle in radians
 */
float Encoder_Right_GetAngleRad(void)
{
    return g_encoder_right.total_revolutions * TWO_PI;
}

/**
 * @brief  Get Left wheel speed in RPM
 * @return Speed in RPM
 */
float Encoder_Left_GetRPM(void)
{
    return g_encoder_left.speed_rpm;
}

/**
 * @brief  Get Right wheel speed in RPM
 * @return Speed in RPM
 */
float Encoder_Right_GetRPM(void)
{
    return g_encoder_right.speed_rpm;
}

/**
 * @brief  Get Left wheel angular velocity in rad/s
 * @return Angular velocity in rad/s
 */
float Encoder_Left_GetRadPerSec(void)
{
    return g_encoder_left.speed_rad_s;
}

/**
 * @brief  Get Right wheel angular velocity in rad/s
 * @return Angular velocity in rad/s
 */
float Encoder_Right_GetRadPerSec(void)
{
    return g_encoder_right.speed_rad_s;
}

/**
 * @brief  Left encoder EXTI callback
 */
void Encoder_Left_EXTI_Callback(void)
{
    uint8_t pin_a = (HAL_GPIO_ReadPin(ENCODER_LEFT_PORT, ENCODER_LEFT_PIN_A) == GPIO_PIN_SET) ? 1U : 0U;
    uint8_t pin_b = (HAL_GPIO_ReadPin(ENCODER_LEFT_PORT, ENCODER_LEFT_PIN_B) == GPIO_PIN_SET) ? 1U : 0U;
    uint8_t current_state = (pin_a << 1U) | pin_b;

    uint8_t index = (g_encoder_left.last_state << 2U) | current_state;
    int8_t step = QUADRATURE_TABLE[index];

    if (step != 0)
    {
        g_encoder_left.total_count += step;
        g_encoder_left.speed_count += step;
    }

    g_encoder_left.last_state = current_state;
}

/**
 * @brief  Right encoder EXTI callback
 */
void Encoder_Right_EXTI_Callback(void)
{
    uint8_t pin_a = (HAL_GPIO_ReadPin(ENCODER_RIGHT_PORT, ENCODER_RIGHT_PIN_A) == GPIO_PIN_SET) ? 1U : 0U;
    uint8_t pin_b = (HAL_GPIO_ReadPin(ENCODER_RIGHT_PORT, ENCODER_RIGHT_PIN_B) == GPIO_PIN_SET) ? 1U : 0U;
    uint8_t current_state = (pin_a << 1U) | pin_b;

    uint8_t index = (g_encoder_right.last_state << 2U) | current_state;
    int8_t step = QUADRATURE_TABLE[index];

    if (step != 0)
    {
        g_encoder_right.total_count += step;
        g_encoder_right.speed_count += step;
    }

    g_encoder_right.last_state = current_state;
}

/***************************************************************************************************************
**                                            EXTI INTERRUPT HANDLERS
***************************************************************************************************************/

/**
 * @brief  EXTI0 IRQ Handler (PB0 / Left Channel A)
 */
void EXTI0_IRQHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(ENCODER_LEFT_PIN_A) != RESET)
    {
        __HAL_GPIO_EXTI_CLEAR_IT(ENCODER_LEFT_PIN_A);
        Encoder_Left_EXTI_Callback();
    }
}

/**
 * @brief  EXTI1 IRQ Handler (PB1 / Left Channel B)
 */
void EXTI1_IRQHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(ENCODER_LEFT_PIN_B) != RESET)
    {
        __HAL_GPIO_EXTI_CLEAR_IT(ENCODER_LEFT_PIN_B);
        Encoder_Left_EXTI_Callback();
    }
}

/**
 * @brief  EXTI15_10 IRQ Handler (PB10 / Right ChA, PB11 / Right ChB)
 */
void EXTI15_10_IRQHandler(void)
{
    if (__HAL_GPIO_EXTI_GET_IT(ENCODER_RIGHT_PIN_A) != RESET)
    {
        __HAL_GPIO_EXTI_CLEAR_IT(ENCODER_RIGHT_PIN_A);
        Encoder_Right_EXTI_Callback();
    }

    if (__HAL_GPIO_EXTI_GET_IT(ENCODER_RIGHT_PIN_B) != RESET)
    {
        __HAL_GPIO_EXTI_CLEAR_IT(ENCODER_RIGHT_PIN_B);
        Encoder_Right_EXTI_Callback();
    }
}
