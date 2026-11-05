/*
****************************************************************************************************************
 * @file       motor.c
 * @brief      Dual DC Motor PWM driver for STM32F1
 *             - Left Motor:  PA0 (TIM2_CH1), PA1 (TIM2_CH2)
 *             - Right Motor: PA2 (TIM2_CH3), PA3 (TIM2_CH4)
 * @date       2026/11/05
 * @author     [Gentantun] (nguyenthanhtung8196@gmail.com)
 * @details    Generates 20kHz PWM on PA0, PA1, PA2, and PA3 using TIM2
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
 * Clock = 72 MHz
 * Prescaler = 0 -> Counter clock = 72 MHz
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
 * @brief  Initialise GPIO PA0..PA3 and TIM2 PWM channels for both Left and Right motors
 */
void Motor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    /* 1. Enable Clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();

    /* 2. Configure PA0, PA1, PA2, PA3 as Alternate Function Push-Pull */
    GPIO_InitStruct.Pin   = MOTOR_LEFT_PIN_IN1  | MOTOR_LEFT_PIN_IN2 |
                            MOTOR_RIGHT_PIN_IN1 | MOTOR_RIGHT_PIN_IN2;
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

    /* 4. Configure PWM Channels 1, 2, 3 & 4 */
    sConfigOC.OCMode       = TIM_OCMODE_PWM1;
    sConfigOC.Pulse        = 0U;
    sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;

    HAL_TIM_PWM_ConfigChannel(&htim2_motor, &sConfigOC, MOTOR_LEFT_TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim2_motor, &sConfigOC, MOTOR_LEFT_TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&htim2_motor, &sConfigOC, MOTOR_RIGHT_TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim2_motor, &sConfigOC, MOTOR_RIGHT_TIM_CHANNEL_2);

    /* 5. Start PWM Outputs (duty = 0) */
    HAL_TIM_PWM_Start(&htim2_motor, MOTOR_LEFT_TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2_motor, MOTOR_LEFT_TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2_motor, MOTOR_RIGHT_TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2_motor, MOTOR_RIGHT_TIM_CHANNEL_2);

    Motor_Stop();
}

/**
 * @brief  Set Left motor speed and direction
 * @param  speed: -1000 to +1000
 */
void Motor_Left_SetSpeed(int16_t speed)
{
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
        /* Forward: IN1 PWM, IN2 LOW */
        uint32_t pulse = (uint32_t)(((uint32_t)speed * MOTOR_TIMER_PERIOD) / MOTOR_PWM_MAX_SPEED);
        __HAL_TIM_SET_COMPARE(&htim2_motor, MOTOR_LEFT_TIM_CHANNEL_1, pulse);
        __HAL_TIM_SET_COMPARE(&htim2_motor, MOTOR_LEFT_TIM_CHANNEL_2, 0U);
    }
    else if (speed < 0)
    {
        /* Reverse: IN1 LOW, IN2 PWM */
        uint32_t pulse = (uint32_t)(((uint32_t)(-speed) * MOTOR_TIMER_PERIOD) / MOTOR_PWM_MAX_SPEED);
        __HAL_TIM_SET_COMPARE(&htim2_motor, MOTOR_LEFT_TIM_CHANNEL_1, 0U);
        __HAL_TIM_SET_COMPARE(&htim2_motor, MOTOR_LEFT_TIM_CHANNEL_2, pulse);
    }
    else
    {
        Motor_Left_Stop();
    }
}

/**
 * @brief  Set Right motor speed and direction
 * @param  speed: -1000 to +1000
 */
void Motor_Right_SetSpeed(int16_t speed)
{
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
        /* Forward: IN1 PWM, IN2 LOW */
        uint32_t pulse = (uint32_t)(((uint32_t)speed * MOTOR_TIMER_PERIOD) / MOTOR_PWM_MAX_SPEED);
        __HAL_TIM_SET_COMPARE(&htim2_motor, MOTOR_RIGHT_TIM_CHANNEL_1, pulse);
        __HAL_TIM_SET_COMPARE(&htim2_motor, MOTOR_RIGHT_TIM_CHANNEL_2, 0U);
    }
    else if (speed < 0)
    {
        /* Reverse: IN1 LOW, IN2 PWM */
        uint32_t pulse = (uint32_t)(((uint32_t)(-speed) * MOTOR_TIMER_PERIOD) / MOTOR_PWM_MAX_SPEED);
        __HAL_TIM_SET_COMPARE(&htim2_motor, MOTOR_RIGHT_TIM_CHANNEL_1, 0U);
        __HAL_TIM_SET_COMPARE(&htim2_motor, MOTOR_RIGHT_TIM_CHANNEL_2, pulse);
    }
    else
    {
        Motor_Right_Stop();
    }
}

/**
 * @brief  Set both motors speed and direction
 * @param  speed_left:  -1000 to +1000
 * @param  speed_right: -1000 to +1000
 */
void Motor_SetSpeed(int16_t speed_left, int16_t speed_right)
{
    Motor_Left_SetSpeed(speed_left);
    Motor_Right_SetSpeed(speed_right);
}

/**
 * @brief  Stop Left motor (Coast)
 */
void Motor_Left_Stop(void)
{
    __HAL_TIM_SET_COMPARE(&htim2_motor, MOTOR_LEFT_TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim2_motor, MOTOR_LEFT_TIM_CHANNEL_2, 0U);
}

/**
 * @brief  Stop Right motor (Coast)
 */
void Motor_Right_Stop(void)
{
    __HAL_TIM_SET_COMPARE(&htim2_motor, MOTOR_RIGHT_TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim2_motor, MOTOR_RIGHT_TIM_CHANNEL_2, 0U);
}

/**
 * @brief  Stop both motors (Coast)
 */
void Motor_Stop(void)
{
    Motor_Left_Stop();
    Motor_Right_Stop();
}

/**
 * @brief  Brake Left motor
 */
void Motor_Left_Brake(void)
{
    __HAL_TIM_SET_COMPARE(&htim2_motor, MOTOR_LEFT_TIM_CHANNEL_1, MOTOR_TIMER_PERIOD);
    __HAL_TIM_SET_COMPARE(&htim2_motor, MOTOR_LEFT_TIM_CHANNEL_2, MOTOR_TIMER_PERIOD);
}

/**
 * @brief  Brake Right motor
 */
void Motor_Right_Brake(void)
{
    __HAL_TIM_SET_COMPARE(&htim2_motor, MOTOR_RIGHT_TIM_CHANNEL_1, MOTOR_TIMER_PERIOD);
    __HAL_TIM_SET_COMPARE(&htim2_motor, MOTOR_RIGHT_TIM_CHANNEL_2, MOTOR_TIMER_PERIOD);
}

/**
 * @brief  Brake both motors
 */
void Motor_Brake(void)
{
    Motor_Left_Brake();
    Motor_Right_Brake();
}
