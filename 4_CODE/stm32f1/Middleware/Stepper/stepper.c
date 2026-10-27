/*
****************************************************************************************************************
 * @file       stepper.c
 * @brief      Stepper motor driver implementation for STM32F103xB using TIM3 PWM
 * @date       2026/10/25
 * @author     [Gentantun] (nguyenthanhtung8196@gmail.com)
 * @details    Controls stepper pulse generation using TIM3 PWM (Channel 1 on PA6) and
 *             direction control via GPIO (PB5). Pulse counting is achieved through the
 *             timer capture-compare 1 (CC1) interrupt.
 * @ref        stm32f7/Middleware/Stepper/stepper.c
 * @copyright  Copyright (c) 2026 RoboTun
****************************************************************************************************************
*/
/***************************************************************************************************************
**                                                   INCLUDES
***************************************************************************************************************/
#include <stdlib.h>
#include <string.h>

#include "stm32f1xx_hal.h"
#include "stepper.h"

/***************************************************************************************************************
**                                          INTERNAL MACRO DEFINITIONS
***************************************************************************************************************/
#define STEP_PORT               GPIOA
#define STEP_PIN                GPIO_PIN_6
#define DIR_PORT                GPIOB
#define DIR_PIN                 GPIO_PIN_5
#define STEP_TIMER              TIM3
#define STEP_CHANNEL            TIM_CHANNEL_1
#define STEP_TIM_IRQn           TIM3_IRQn

/***************************************************************************************************************
**                                        INTERNAL VARIABLE DEFINITIONS
***************************************************************************************************************/
static TIM_HandleTypeDef htim3;
static stepper_t *g_stepper = NULL;
static volatile uint32_t target_steps = 0U;
static volatile uint32_t completed_steps = 0U;
static volatile uint8_t  busy = 0U;

/***************************************************************************************************************
**                                         INTERNAL FUNCTION PROTOTYPES
***************************************************************************************************************/
static void     stepper_init(void);
static void     stepper_set_direction(uint8_t dir);
static void     stepper_set_frequency(uint32_t frequency_hz);
static void     stepper_start(void);
static void     stepper_stop(void);
static void     stepper_move_steps(uint32_t steps, uint8_t dir, uint32_t frequency_hz);
static uint8_t  stepper_is_busy(void);

/***************************************************************************************************************
**                                             FUNCTION DEFINITIONS
***************************************************************************************************************/

/**
 * @brief  Initialise GPIO pins and TIM3 PWM for stepper generation.
 */
static void stepper_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    TIM_OC_InitTypeDef pwm = {0};
    RCC_ClkInitTypeDef clocks = {0};
    uint32_t flash_latency = 0U;
    uint32_t timer_clock = 0U;

    /* Enable peripheral clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();

    /* Configure DIR pin as push-pull output (PB5) */
    gpio.Pin   = DIR_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DIR_PORT, &gpio);
    HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_RESET);

    /* Configure STEP pin as Alternate Function Push-Pull (PA6 = TIM3_CH1 on F1) */
    gpio.Pin   = STEP_PIN;
    gpio.Mode  = GPIO_MODE_AF_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(STEP_PORT, &gpio);

    /* Determine TIM3 input clock frequency (PCLK1 x 2 when APB1 prescaler != 1) */
    HAL_RCC_GetClockConfig(&clocks, &flash_latency);
    timer_clock = HAL_RCC_GetPCLK1Freq();
    if (clocks.APB1CLKDivider != RCC_HCLK_DIV1)
    {
        timer_clock *= 2U;
    }

    /* Configure TIM3 time base for 1 MHz resolution (1 tick = 1 microsecond) */
    htim3.Instance               = STEP_TIMER;
    htim3.Init.Prescaler         = (timer_clock / 1000000U) - 1U;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 999U;  /* Default 1 kHz */
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    (void)HAL_TIM_PWM_Init(&htim3);

    /* Configure PWM mode 1 with 50% duty cycle default */
    pwm.OCMode       = TIM_OCMODE_PWM1;
    pwm.Pulse        = 500U;
    pwm.OCPolarity   = TIM_OCPOLARITY_HIGH;
    pwm.OCFastMode   = TIM_OCFAST_DISABLE;
    (void)HAL_TIM_PWM_ConfigChannel(&htim3, &pwm, STEP_CHANNEL);

    /* Configure NVIC for step counting interrupt */
    HAL_NVIC_SetPriority(STEP_TIM_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(STEP_TIM_IRQn);

    if (g_stepper != NULL)
    {
        g_stepper->stepper_reg.bit.enable = 0U;
        g_stepper->stepper_reg.bit.dir    = STEPPER_STOP;
        g_stepper->stepper_reg.bit.status = 0U;
    }
}

/**
 * @brief  Set stepper motor direction.
 * @param  dir  STEPPER_FORWARD or STEPPER_REVERSE.
 */
static void stepper_set_direction(uint8_t dir)
{
    const uint8_t forward = (dir == STEPPER_FORWARD);
    HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
    if (g_stepper != NULL)
    {
        g_stepper->stepper_reg.bit.dir = forward ? STEPPER_FORWARD : STEPPER_REVERSE;
    }
}

/**
 * @brief  Set step PWM frequency in Hertz.
 * @param  frequency_hz  Target pulse frequency (Hz).
 */
static void stepper_set_frequency(uint32_t frequency_hz)
{
    uint32_t period;
    if ((frequency_hz == 0U) || (htim3.Instance == NULL))
    {
        return;
    }

    period = 1000000U / frequency_hz;
    if (period < 2U)
    {
        period = 2U;
    }
    __HAL_TIM_SET_AUTORELOAD(&htim3, period - 1U);
    __HAL_TIM_SET_COMPARE(&htim3, STEP_CHANNEL, period / 2U);

    if (g_stepper != NULL)
    {
        g_stepper->stepper_reg.bit.speed = (frequency_hz > 127U) ? 127U : frequency_hz;
    }
}

/**
 * @brief  Start continuous step pulse generation.
 */
static void stepper_start(void)
{
    (void)HAL_TIM_PWM_Start(&htim3, STEP_CHANNEL);
    if (g_stepper != NULL)
    {
        g_stepper->stepper_reg.bit.enable = 1U;
    }
}

/**
 * @brief  Stop step pulse generation.
 */
static void stepper_stop(void)
{
    (void)HAL_TIM_PWM_Stop(&htim3, STEP_CHANNEL);
    __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_CC1);
    busy = 0U;
    if (g_stepper != NULL)
    {
        g_stepper->stepper_reg.bit.enable = 0U;
        g_stepper->stepper_reg.bit.dir    = STEPPER_STOP;
    }
}

/**
 * @brief  Command a discrete number of steps with direction and frequency.
 * @param  steps         Number of steps to execute.
 * @param  dir           Direction (STEPPER_FORWARD or STEPPER_REVERSE).
 * @param  frequency_hz  Step pulse rate in Hz.
 */
static void stepper_move_steps(uint32_t steps, uint8_t dir, uint32_t frequency_hz)
{
    if ((steps == 0U) || (frequency_hz == 0U) || busy || (htim3.Instance == NULL))
    {
        return;
    }

    stepper_set_direction(dir);
    stepper_set_frequency(frequency_hz);
    target_steps    = steps;
    completed_steps = 0U;
    busy            = 1U;

    __HAL_TIM_SET_COUNTER(&htim3, 0U);
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_CC1);
    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC1);
    stepper_start();
}

/**
 * @brief  Check whether the stepper motor is actively executing steps.
 * @return 1 if busy, 0 otherwise.
 */
static uint8_t stepper_is_busy(void)
{
    return busy;
}

/**
 * @brief  Factory function allocating and initializing a stepper driver instance.
 * @return Pointer to initialized stepper_t structure, or NULL on failure.
 */
stepper_t *stepper_create(void)
{
    stepper_t *stepper = (stepper_t *)malloc(sizeof(*stepper));
    if (stepper == NULL)
    {
        return NULL;
    }

    memset(stepper, 0, sizeof(*stepper));
    stepper->stepper_init          = stepper_init;
    stepper->stepper_set_direction = stepper_set_direction;
    stepper->stepper_set_frequency = stepper_set_frequency;
    stepper->stepper_start         = stepper_start;
    stepper->stepper_stop          = stepper_stop;
    stepper->stepper_move_steps    = stepper_move_steps;
    stepper->stepper_is_busy       = stepper_is_busy;
    g_stepper = stepper;
    return stepper;
}

/**
 * @brief  TIM3 global interrupt handler for step pulse counting.
 */
void TIM3_IRQHandler(void)
{
    if ((__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_CC1) != RESET) &&
        (__HAL_TIM_GET_IT_SOURCE(&htim3, TIM_IT_CC1) != RESET))
    {
        __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_CC1);
        if (busy && (++completed_steps >= target_steps))
        {
            stepper_stop();
        }
    }
}

/***************************************************************************************************************
**                                                End of file
***************************************************************************************************************/
