/*
****************************************************************************************************************
 * @file       main.c
 * @brief      Application entry point for STM32F103xB (Cortex-M3) with Azure RTOS ThreadX
 * @date       2026/10/25
 * @author     [Gentantun] (nguyenthanhtung8196@gmail.com)
 * @details    Initialises hardware (72 MHz HSE + PLL), user LED on PC13, debug UART on PA9,
 *             stepper motor driver on TIM3 CH1 / PB5, and enters the ThreadX kernel.
 *             Two threads are created:
 *               - thread_0: primary application & stepper motor actuation loop
 *               - thread_1: background housekeeping tasks
 * @ref        stm32f7/Application/main.c (original template)
 * @copyright  Copyright (c) 2026 RoboTun
****************************************************************************************************************
*/
/***************************************************************************************************************
**                                                   INCLUDES
***************************************************************************************************************/
#include "main.h"
#include "tx_api.h"
#include "debug_uart.h"
#include "accelero_gyro.h"
#include "encoder.h"
#include "motor.h"
#include "pid.h"

/***************************************************************************************************************
**                                          INTERNAL MACRO DEFINITIONS
***************************************************************************************************************/
/* ThreadX resource sizing */
#define DEMO_STACK_SIZE         1024U       /* Stack size per thread (bytes) */
#define DEMO_BYTE_POOL_SIZE     9120U       /* Total byte pool for all stacks */

/* Control loop sampling period: 200Hz = 5ms */
#define CONTROL_LOOP_FREQ_HZ    200U
#define CONTROL_LOOP_DT_SEC     (1.0f / (float)CONTROL_LOOP_FREQ_HZ)

/* Safety limits for robot balancing */
#define MAX_TILT_ANGLE_DEG      40.0f       /* Disengage motors if tilted beyond +/- 40 degrees */
#define MECHANICAL_ZERO_ANGLE   0.0f        /* Natural balance center-of-mass trim angle in degrees */

/***************************************************************************************************************
**                                        INTERNAL VARIABLE DEFINITIONS
***************************************************************************************************************/

static TX_BYTE_POOL byte_pool_0;
static TX_THREAD    thread_0;
static TX_THREAD    thread_1;
static UCHAR        memory_area[DEMO_BYTE_POOL_SIZE];

/* IMU & Control instances */
static MPU6500_driver_t *mpu = NULL;
static imu_sensor_t      imu;
static TIM_HandleTypeDef htim4;

/* PID controllers */
static pid_controller_t pid_balance;
static pid_controller_t pid_velocity;
static volatile int16_t g_motor_output = 0;

/***************************************************************************************************************
**                                         INTERNAL FUNCTION PROTOTYPES
***************************************************************************************************************/
static void SystemClock_Config(void);
static void Error_Handler(void);
static void Robot_Timer4_Init(void);
static void Robot_Setup(void);

/* ThreadX thread entry functions */
static void thread_0_entry(ULONG thread_input);
static void thread_1_entry(ULONG thread_input);

/***************************************************************************************************************
**                                             FUNCTION DEFINITIONS
***************************************************************************************************************/

/**
 * @brief  Initialise TIM4 as periodic interrupt timer for the balancing loop (200Hz = 5ms)
 *         APB1 Timer Clock = 72 MHz
 *         Prescaler = 72 - 1 -> Timer clock = 1 MHz (1 tick = 1 us)
 *         Period = 5000 - 1  -> 5000 us = 5 ms (200 Hz)
 */
static void Robot_Timer4_Init(void)
{
    __HAL_RCC_TIM4_CLK_ENABLE();

    htim4.Instance               = TIM4;
    htim4.Init.Prescaler         = 72U - 1U;
    htim4.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim4.Init.Period            = 5000U - 1U;
    htim4.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
    {
        Error_Handler();
    }

    /* Configure TIM4 interrupt priority and enable IRQ */
    HAL_NVIC_SetPriority(TIM4_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
}

/**
 * @brief  Hardware and sensor setup routine for self-balancing robot
 */
static void Robot_Setup(void)
{
    /* 1. Initialize Motor Driver first (PA0, PA1 - TIM2 PWM) */
    Motor_Init();
    Motor_Stop();
    TRACE_INFO("Motor driver (PA0, PA1) initialized.\n");

    /* 2. Initialize Encoder on PB0 & PB1 */
    Encoder_Init();
    TRACE_INFO("Encoder (PB0, PB1) initialized.\n");

    /* 3. Initialize MPU6500 */
    TRACE_INFO("Initializing MPU6500...\n");
    mpu = MPU6500_driver_create();
    if (mpu == NULL)
    {
        TRACE_INFO("Failed to create MPU6500 driver!\n");
        Error_Handler();
    }

    if (mpu->init(mpu) != HAL_OK)
    {
        TRACE_INFO("MPU6500 init failed!\n");
        Error_Handler();
    }
    TRACE_INFO("MPU6500 initialized successfully.\n");

    /* 4. Initialize IMU container with filters */
    IMU_Init(&imu, mpu);

    /* 5. Calibrate sensors: Keep robot stationary on flat surface! */
    TRACE_INFO("Calibrating IMU (keep stationary)...\n");
    IMU_Calibrate(&imu, 500);
    TRACE_INFO("IMU Calibration done. Ready!\n");

    /* 6. Initialize PID controllers
     * Balance PD: Kp=30.0f, Ki=0.0f, Kd=1.2f, Limits: [-1000, 1000]
     * Velocity PI: Kp=0.2f, Ki=0.02f, Kd=0.0f, Tilt adjustment limits: [-5.0 deg, +5.0 deg]
     */
    PID_Init(&pid_balance, 30.0f, 0.0f, 1.2f, 0.0f, -1000.0f, 1000.0f);
    PID_Init(&pid_velocity, 0.2f, 0.02f, 0.0f, 5.0f, -5.0f, 5.0f);

    /* 7. Initialize TIM4 for 200Hz control loop */
    Robot_Timer4_Init();

    /* Start TIM4 Base Interrupt */
    HAL_TIM_Base_Start_IT(&htim4);
    TRACE_INFO("Balancing timer TIM4 (200Hz) started.\n");
}

/**
 * @brief  TIM4 Interrupt Handler - Executes at 200Hz (every 5ms)
 */
void TIM4_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_UPDATE) != RESET)
    {
        if (__HAL_TIM_GET_IT_SOURCE(&htim4, TIM_IT_UPDATE) != RESET)
        {
            __HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);

            /* 200Hz IMU update & filter calculation */
            if (imu.is_calibrated)
            {
                IMU_Update(&imu, CONTROL_LOOP_DT_SEC);
                Encoder_Update(CONTROL_LOOP_DT_SEC);

                /* Safety check: Stop motors if tilted beyond allowable angle */
                if (imu.kalman_pitch > MAX_TILT_ANGLE_DEG || imu.kalman_pitch < -MAX_TILT_ANGLE_DEG)
                {
                    Motor_Stop();
                    PID_Reset(&pid_balance);
                    PID_Reset(&pid_velocity);
                    g_motor_output = 0;
                }
                else
                {
                    /* 1. Velocity loop: compute target tilt offset angle */
                    float target_angle = PID_VelocityUpdate(&pid_velocity, 
                                                            Encoder_GetRPM(), 
                                                            0.0f, 
                                                            CONTROL_LOOP_DT_SEC);
                    target_angle += MECHANICAL_ZERO_ANGLE;

                    /* 2. Balance loop: compute motor PWM command using direct gyro rate (imu.gy) */
                    float output = PID_BalanceUpdate(&pid_balance, 
                                                     imu.kalman_pitch, 
                                                     target_angle, 
                                                     imu.gy, 
                                                     CONTROL_LOOP_DT_SEC);

                    g_motor_output = (int16_t)output;
                    Motor_SetSpeed(g_motor_output);
                }
            }
        }
    }
}

/**
 * @brief  Initialise user LED (PC13 on Blue-Pill board, active-LOW).
 */
void led_init(void)
{
    GPIO_InitTypeDef gpio_init = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();

    gpio_init.Pin   = GPIO_PIN_13;
    gpio_init.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull  = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &gpio_init);

    /* Turn LED off at start (PC13 active-LOW) */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
}

/**
 * @brief  Toggle user LED.
 */
void toggle_led(void)
{
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
}

/**
 * @brief  Application entry point.
 * @return Never returns (ThreadX scheduler takes over).
 */
int main(void)
{
    /* Initialise HAL */
    HAL_Init();

    /* Configure system clock: HSE 8 MHz -> PLL -> 72 MHz SYSCLK */
    SystemClock_Config();

    /* Peripheral initialisations */
    led_init();
    debug_uart_init();
    TRACE_INFO("STM32F1 Application started\n");

    /* Hand control to the ThreadX kernel — never returns */
    tx_kernel_enter();

    /* Infinite loop fallback */
    while (1)
    {
        // toggle_led();
        // HAL_Delay(500U);
    }
}

/**
 * @brief  ThreadX application-define callback.
 *         Creates memory byte pool and initial threads.
 * @param  first_unused_memory  Passed by ThreadX low-level init.
 */
void tx_application_define(void *first_unused_memory)
{
    CHAR *pointer = TX_NULL;
    (void)first_unused_memory;

    /* Create byte memory pool */
    tx_byte_pool_create(&byte_pool_0, "byte pool 0", memory_area, DEMO_BYTE_POOL_SIZE);

    /* Create Thread 0 (Application logic) */
    tx_byte_allocate(&byte_pool_0, (VOID **)&pointer, DEMO_STACK_SIZE, TX_NO_WAIT);
    tx_thread_create(&thread_0, "thread 0", thread_0_entry, 0UL,
                     pointer, DEMO_STACK_SIZE,
                     1U, 1U, TX_NO_TIME_SLICE, TX_AUTO_START);

    /* Create Thread 1 (Housekeeping) */
    tx_byte_allocate(&byte_pool_0, (VOID **)&pointer, DEMO_STACK_SIZE, TX_NO_WAIT);
    tx_thread_create(&thread_1, "thread 1", thread_1_entry, 1UL,
                     pointer, DEMO_STACK_SIZE,
                     16U, 16U, 4U, TX_AUTO_START);

    TRACE_INFO("OS init done.\n");
}

/**
 * @brief  Thread 0 body — primary application & stepper actuation logic.
 * @param  thread_input  Thread parameter.
 */
static void thread_0_entry(ULONG thread_input)
{
    (void)thread_input;

    /* Initialize IMU, calibrate and start 200Hz balancing timer */
    Robot_Setup();

    // Motor_SetSpeed(1000);

    while (1)
    {
        /* 1. Robot pitch attitude (Kalman Filter) */
        float k_pitch = imu.kalman_pitch;
        int32_t k_int  = (int32_t)k_pitch;
        int32_t k_frac = (int32_t)((k_pitch < 0 ? -k_pitch : k_pitch) * 100.0f) % 100;
        const char *k_sign = (k_pitch < 0 && k_int == 0) ? "-" : "";

        /* 2. Wheel rotation angle from encoder */
        float enc_deg = Encoder_GetAngleDeg();
        int32_t enc_deg_int  = (int32_t)enc_deg;
        int32_t enc_deg_frac = (int32_t)((enc_deg < 0 ? -enc_deg : enc_deg) * 100.0f) % 100;
        const char *enc_sign = (enc_deg < 0 && enc_deg_int == 0) ? "-" : "";

        /* Output robot pitch, wheel angle, tick counts, RPM, and motor command */
        TRACE_INFO("Pitch: %s%ld.%02ld deg | Wheel: %s%ld.%02ld deg | RPM: %ld | Motor: %d\n", 
                   k_sign, (long)k_int, (long)k_frac, 
                   enc_sign, (long)enc_deg_int, (long)enc_deg_frac,
                   (long)Encoder_GetRPM(), g_motor_output);

        tx_thread_sleep(10U);
    }
}

/**
 * @brief  Thread 1 body — background housekeeping tasks.
 * @param  thread_input  Thread parameter.
 */
static void thread_1_entry(ULONG thread_input)
{
    (void)thread_input;

    while (1)
    {
        if (!imu.is_calibrated)
        {
            /* During calibration: blink LED, keep motor stationary */
            toggle_led();
            tx_thread_sleep(10U);
        }
        else
        {
            /* Calibration completed: turn off LED */
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
            tx_thread_sleep(50U);
        }
    }
}

/**
 * @brief  System clock configuration for STM32F103xB:
 *         SYSCLK = 72 MHz (HSE 8 MHz x 9)
 *         AHB    = 72 MHz (DIV1)
 *         APB1   = 36 MHz (DIV2, max 36 MHz)
 *         APB2   = 72 MHz (DIV1)
 */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState       = RCC_HSI_OFF;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL9;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType      = (RCC_CLOCKTYPE_SYSCLK |
                                        RCC_CLOCKTYPE_HCLK   |
                                        RCC_CLOCKTYPE_PCLK1  |
                                        RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief  Generic error handler.
 */
static void Error_Handler(void)
{
    while (1)
    {
    }
}

/***************************************************************************************************************
**                                                End of file
***************************************************************************************************************/
