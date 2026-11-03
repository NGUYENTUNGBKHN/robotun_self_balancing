/*
****************************************************************************************************************
 * @file       motor.c
 * @brief      DC Motor PWM driver for STM32F1 (PA0: IN1 / TIM2_CH1, PA1: IN2 / TIM2_CH2)
 * @date       2026/11/03
 * @author     [Gentantun] (nguyenthanhtung8196@gmail.com)
 * @details    Generates 20kHz PWM on PA0 and PA1 to drive H-Bridge
 * @copyright  Copyright (c) 2026 RoboTun
****************************************************************************************************************
*/

/***************************************************************************************************************
**                                                   INCLUDES
***************************************************************************************************************/
#include "motor.h"

/***************************************************************************************************************
**                                          INTERNAL MACRO DEFINITIONS
***************************************************************************************************************/
/* Timer configuration:
 * Input clock = 72 MHz
 * Prescaler = 0 -> Timer counter clock = 72 MHz
 * ARR = 3599 -> PWM frequency = 72,000,000 / (1 * 3600) = 20,000 Hz (20 kHz)
 */
#define MOTOR_TIMER_PERIOD          (3599U)

/***************************************************************************************************************
**                                        INTERNAL VARIABLE DEFINITIONS
***************************************************************************************************************/
static TIM_HandleTypeDef htim2_motor;

/***************************************************************************************************************
**                                             FUNCTION DEFINITIONS
***************************************************************************************************************/

/**
 * @brief  Initialise GPIO PA0, PA1 and TIM2 PWM channels for Motor Driver
 */
void Motor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    /* 1. Enable Clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();

    /* 2. Configure PA0 and PA1 as Alternate Function Push-Pull */
    GPIO_InitStruct.Pin   = MOTOR_PIN_IN1 | MOTOR_PIN_IN2;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(MOTOR_PORT, &GPIO_InitStruct);

    /* 3. Configure TIM2 Time Base: 20kHz PWM */
    htim2_motor.Instance               = MOTOR_TIM_INSTANCE;
    htim2_motor.Init.Prescaler         = 0U;
    htim2_motor.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2_motor.Init.Period            = MOTOR_TIMER_PERIOD;
    htim2_motor.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2_motor.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    HAL_TIM_PWM_Init(&htim2_motor);

    /* 4. Configure PWM Channels 1 & 2 */
    sConfigOC.OCMode       = TIM_OCMODE_PWM1;
    sConfigOC.Pulse        = 0U;
    sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;

    HAL_TIM_PWM_ConfigChannel(&htim2_motor, &sConfigOC, MOTOR_TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim2_motor, &sConfigOC, MOTOR_TIM_CHANNEL_2);

    /* 5. Start PWM Outputs (duty = 0) */
    HAL_TIM_PWM_Start(&htim2_motor, MOTOR_TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2_motor, MOTOR_TIM_CHANNEL_2);

    Motor_Stop();
}

/**
 * @brief  Set motor speed and direction
 * @param  speed: -1000 to +1000
 */
void Motor_SetSpeed(int16_t speed)
{
    /* Clamp speed range */
    if (speed > MOTOR_PWM_MAX_SPEED)
    {
        speed = MOTOR_PWM_MAX_SPEED;
    }
    else if (speed < -MOTOR_PWM_MAX_SPEED)
    {
        speed = -MOTOR_PWM_MAX_SPEED;
    }

    if (speed > 0)
    {
        /* Forward: PA0 (CH1) PWM, PA1 (CH2) LOW */
        uint32_t pulse = (uint32_t)(((uint32_t)speed * MOTOR_TIMER_PERIOD) / MOTOR_PWM_MAX_SPEED);
        __HAL_TIM_SET_COMPARE(&htim2_motor, MOTOR_TIM_CHANNEL_1, pulse);
        __HAL_TIM_SET_COMPARE(&htim2_motor, MOTOR_TIM_CHANNEL_2, 0U);
    }
    else if (speed < 0)
    {
        /* Reverse: PA0 (CH1) LOW, PA1 (CH2) PWM */
        uint32_t pulse = (uint32_t)(((uint32_t)(-speed) * MOTOR_TIMER_PERIOD) / MOTOR_PWM_MAX_SPEED);
        __HAL_TIM_SET_COMPARE(&htim2_motor, MOTOR_TIM_CHANNEL_1, 0U);
        __HAL_TIM_SET_COMPARE(&htim2_motor, MOTOR_TIM_CHANNEL_2, pulse);
    }
    else
    {
        /* Coast / Stop: Both 0 */
        Motor_Stop();
    }
}

/**
 * @brief  Stop motor (Coast)
 */
void Motor_Stop(void)
{
    __HAL_TIM_SET_COMPARE(&htim2_motor, MOTOR_TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim2_motor, MOTOR_TIM_CHANNEL_2, 0U);
}

/**
 * @brief  Brake motor
 */
void Motor_Brake(void)
{
    __HAL_TIM_SET_COMPARE(&htim2_motor, MOTOR_TIM_CHANNEL_1, MOTOR_TIMER_PERIOD);
    __HAL_TIM_SET_COMPARE(&htim2_motor, MOTOR_TIM_CHANNEL_2, MOTOR_TIMER_PERIOD);
}

