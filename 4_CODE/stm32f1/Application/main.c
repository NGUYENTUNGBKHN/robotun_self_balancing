/*
****************************************************************************************************************
 * @file       main.c
 * @brief      Application entry point for STM32F103xB (Cortex-M3) with Azure RTOS ThreadX
 * @date       2026/11/05
 * @author     [Gentantun] (nguyenthanhtung8196@gmail.com)
 * @details    Initialises hardware (72 MHz HSE + PLL), user LED on PC13, debug UART on PA9,
 *             dual DC motor driver (TIM2 CH1-CH4), dual quadrature encoders (PB0/PB1 and PB10/PB11),
 *             MPU6500 IMU, and enters the ThreadX kernel.
 *             Two threads are created:
 *               - thread_0: robot setup, then USB telemetry transmit
 *               - thread_1: background housekeeping tasks (LED indicator)
 *             Balancing loop runs in TIM4 interrupt at 200Hz with 3-stage cascade:
 *               Position Loop -> Velocity Loop -> Balance Loop.
 * @copyright  Copyright (c) 2026 RoboTunP
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
#include "usb_device.h"
#include "usbd_cdc_if.h"
/***************************************************************************************************************
**                                          INTERNAL MACRO DEFINITIONS
***************************************************************************************************************/
/* ThreadX resource sizing */
#define THREAD0_STACK_SIZE      2048U       /* thread_0 calls Robot_Setup + TRACE_INFO (printf) */
#define THREAD1_STACK_SIZE      1024U
#define DEMO_BYTE_POOL_SIZE     9120U       /* Total byte pool for all stacks */

/* Control loop sampling period: 200Hz = 5ms */
#define CONTROL_LOOP_FREQ_HZ    200U
#define CONTROL_LOOP_DT_SEC     (1.0f / (float)CONTROL_LOOP_FREQ_HZ)

/* Decimation factors: decouple inner high-speed balance loop from outer mechanical loops */
#define BALANCE_LOOP_DIVIDER    1U          /* Balance loop runs with the 200Hz IMU loop */
#define VELOCITY_LOOP_DIVIDER   4U          /* 200Hz / 4 = 50Hz (every 20ms) */
#define POSITION_LOOP_DIVIDER   8U          /* 200Hz / 8 = 25Hz (every 40ms) */

/* Safety limits for robot balancing */
#define MAX_TILT_ANGLE_DEG      30.0f       /* Disengage motors if tilted beyond +/- 30 degrees */
#define MECHANICAL_ZERO_ANGLE   0.0f        /* Natural balance center-of-mass trim angle in degrees */

/* Re-arm after a fall: |pitch| < 3 deg continuously for 0.5 s (100 ticks x 5 ms) */
#define REARM_ANGLE_DEG         3.0f
#define REARM_TICKS             100U

/* Command limits for the position -> velocity -> pitch cascade */
#define TARGET_POSITION_LIMIT_DEG   30.0f   /* Position command range: +/- 30 degrees */
#define TARGET_VELOCITY_LIMIT_RPM   60.0f   /* Position loop output (wheel speed): +/- 60 RPM */
#define TARGET_PITCH_LIMIT_DEG      5.0f    /* Velocity loop output (balance target): +/- 5 degrees */

/* Motor output & deadzone compensation (continuous, no jump around 0)
 *   |out| in [0 .. RAMP]     -> [0 .. DZ_EFF]
 *   |out| in [RAMP .. LIMIT] -> [DZ_EFF .. LIMIT]
 */
#define PID_OUTPUT_LIMIT        1000.0f
#define MOTOR_DEADZONE_EFF      170.0f      /* ~60% of measured 280, tune on the real robot */
#define MOTOR_DEADZONE_RAMP     100.0f

/* Balance loop (Fastest - 200Hz) */
#define PID_BALANCE_KP          300.0f
#define PID_BALANCE_KI          0.0f
#define PID_BALANCE_KD          0.5f

/* Velocity loop (50Hz): Gently regulates wheel speed to stay in place */
#define PID_VELOCITY_KP         0.0f
#define PID_VELOCITY_KI         0.0f
#define PID_VELOCITY_KD         0.0f

/* Position loop (25Hz): Locks position around target point */
#define PID_POSITION_KP         0.0f
#define PID_POSITION_KI         0.0f
#define PID_POSITION_KD         0.0f

/***************************************************************************************************************
**                                             INTERNAL TYPE DEFINITIONS
***************************************************************************************************************/
typedef struct __attribute__((packed)) USB_PACKAGE_S
{
    uint8_t header;
    uint8_t length;
    int16_t accel[3];          /* Ax, Ay, Az scaled by 1000 */
    int16_t gyro[3];           /* Gx, Gy, Gz scaled by 100 deg/s */
    int16_t pitch_x100;        /* Kalman pitch * 100 */
    int16_t target_x100;       /* Balance target angle * 100 */
    int16_t left_rpm_x100;     /* Left wheel RPM * 100 */
    int16_t right_rpm_x100;    /* Right wheel RPM * 100 */
    int16_t position_x10;      /* Average wheel position * 10 */
    int16_t motor_output;      /* Motor PWM command */
    uint8_t end;
} usb_package_t;

/***************************************************************************************************************
**                                        INTERNAL VARIABLE DEFINITIONS
***************************************************************************************************************/
static usb_package_t robot_data;

static TX_BYTE_POOL byte_pool_0;
static TX_THREAD    thread_0;
static TX_THREAD    thread_1;
static UCHAR        memory_area[DEMO_BYTE_POOL_SIZE];

/* IMU & Control instances */
static MPU6500_driver_t *mpu = NULL;
static imu_sensor_t      imu;
static TIM_HandleTypeDef htim4;

/* PID controllers: 3-stage cascade (Position -> Velocity -> Balance) */
static pid_controller_t pid_balance;
static pid_controller_t pid_velocity;
static pid_controller_t pid_position;

static volatile int16_t g_motor_output      = 0;
static float            g_target_position   = 0.0f;  /* Target wheel position in degrees (0 = lock position) */
static float            g_target_rpm        = 0.0f;  /* Target speed from position loop */
static float            g_target_angle      = 0.0f;  /* Target angle from velocity loop (without zero trim) */
static volatile float   g_balance_target    = MECHANICAL_ZERO_ANGLE; /* Angle actually fed to balance loop */
static uint8_t          g_loop_counter      = 0U;    /* Decimation counter for cascade loops */
static uint8_t          g_telemetry_counter = 0U;    /* 50Hz USB telemetry decimation */

/* Fall latch: 1 = motors locked (also at power-up: hold robot upright to arm) */
static volatile uint8_t g_fallen            = 1U;
static uint16_t         g_rearm_counter     = 0U;
static volatile uint8_t g_telemetry_pending = 0U;

/***************************************************************************************************************
**                                         INTERNAL FUNCTION PROTOTYPES
***************************************************************************************************************/
static void SystemClock_Config(void);
static void Robot_Timer4_Init(void);
static void Robot_Setup(void);

/* ThreadX thread entry functions */
static void thread_0_entry(ULONG thread_input);
static void thread_1_entry(ULONG thread_input);

/***************************************************************************************************************
**                                             FUNCTION DEFINITIONS
***************************************************************************************************************/
/**
 * @brief  Force USB re-enumeration by pulling DP (PA12) LOW briefly.
 */
static void USB_Reset_DP(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin   = GPIO_PIN_12;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_Delay(50);
}

/**
 * @brief  Saturate a float to int16 range.
 */
static inline int16_t sat_i16(float v)
{
    if (v >  32767.0f) { return  32767; }
    if (v < -32768.0f) { return -32768; }
    return (int16_t)v;
}

/**
 * @brief  Continuous deadzone compensation (no step around 0).
 * @param  out  PID output in [-PID_OUTPUT_LIMIT, +PID_OUTPUT_LIMIT]
 * @return Motor command with static-friction offset applied.
 */
static int16_t Balance_CompensateDeadzone(float out)
{
    float mag = (out < 0.0f) ? -out : out;
    float res;

    if (mag >= PID_OUTPUT_LIMIT)
    {
        res = PID_OUTPUT_LIMIT;
    }
    else if (mag < MOTOR_DEADZONE_RAMP)
    {
        res = mag * (MOTOR_DEADZONE_EFF / MOTOR_DEADZONE_RAMP);
    }
    else
    {
        res = MOTOR_DEADZONE_EFF +
              (mag - MOTOR_DEADZONE_RAMP) *
              ((PID_OUTPUT_LIMIT - MOTOR_DEADZONE_EFF) /
               (PID_OUTPUT_LIMIT - MOTOR_DEADZONE_RAMP));
    }
    return (int16_t)((out < 0.0f) ? -res : res);
}

/**
 * @brief  Reset all controller state.
 */
static void Balance_ResetControl(void)
{
    PID_Reset(&pid_balance);
    PID_Reset(&pid_velocity);
    PID_Reset(&pid_position);
    g_target_rpm     = 0.0f;
    g_target_angle   = 0.0f;
    g_balance_target = MECHANICAL_ZERO_ANGLE;
    g_loop_counter   = 0U;
}

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
    /* 1. Initialize Dual Motor Driver (Left: PA0/PA1, Right: PA2/PA3 - TIM2 PWM) */
    Motor_Init();
    Motor_Stop();
    TRACE_INFO("Dual Motor driver initialized.\n");

    /* 2. Initialize Dual Encoders (Left: PB0/PB1, Right: PB10/PB11) */
    Encoder_Init();
    TRACE_INFO("Dual Encoders (Left PB0/PB1, Right PB10/PB11) initialized.\n");

    /* 3. Initialize MPU6500 */
    TRACE_INFO("Initializing MPU6500...\n");
    mpu = MPU6500_driver_create();
    if (mpu == NULL)
    {
        TRACE_INFO("Failed to create MPU6500 driver!\n");
        Error_Handler();
    }

    if (mpu->init(mpu) != ACE_SUCCESS)
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

    /* 6. Initialize PID controllers (output limits come from the macros above) */
    PID_Init(&pid_balance,  PID_BALANCE_KP,  PID_BALANCE_KI,  PID_BALANCE_KD,
             0.0f,  -PID_OUTPUT_LIMIT,          PID_OUTPUT_LIMIT);
    PID_Init(&pid_velocity, PID_VELOCITY_KP, PID_VELOCITY_KI, PID_VELOCITY_KD,
             5.0f,  -TARGET_PITCH_LIMIT_DEG,    TARGET_PITCH_LIMIT_DEG);
    PID_Init(&pid_position, PID_POSITION_KP, PID_POSITION_KI, PID_POSITION_KD,
             10.0f, -TARGET_VELOCITY_LIMIT_RPM, TARGET_VELOCITY_LIMIT_RPM);

    /* 7. Initialize TIM4 for 200Hz control loop */
    Robot_Timer4_Init();

    /* Start TIM4 Base Interrupt */
    HAL_TIM_Base_Start_IT(&htim4);
    TRACE_INFO("Balancing timer TIM4 (200Hz) started. Hold robot upright to arm.\n");
}

/**
 * @brief  TIM4 Interrupt Handler - Executes at 200Hz (every 5ms)
 */
void TIM4_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_UPDATE) != RESET &&
        __HAL_TIM_GET_IT_SOURCE(&htim4, TIM_IT_UPDATE) != RESET)
    {
        __HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);

        /* 200Hz IMU update & filter calculation */
        if (imu.is_calibrated)
        {
            IMU_Update(&imu, CONTROL_LOOP_DT_SEC);
            Encoder_Update(CONTROL_LOOP_DT_SEC);

            float pitch     = imu.kalman_pitch;
            float abs_pitch = (pitch < 0.0f) ? -pitch : pitch;

            if (g_fallen)
            {
                /* ---- Locked: motors off, wait until robot is held upright ---- */
                Motor_Stop();
                g_motor_output = 0;

                if (abs_pitch < REARM_ANGLE_DEG)
                {
                    if (++g_rearm_counter >= REARM_TICKS)
                    {
                        Balance_ResetControl();
                        Encoder_Reset();            /* Zero wheel position (add to encoder.c if missing) */
                        g_rearm_counter = 0U;
                        g_fallen        = 0U;
                    }
                }
                else
                {
                    g_rearm_counter = 0U;
                }
            }
            else if (abs_pitch > MAX_TILT_ANGLE_DEG)
            {
                /* ---- Just fell: lock motors until re-armed ---- */
                Motor_Stop();
                g_motor_output = 0;
                Balance_ResetControl();
                g_rearm_counter = 0U;
                g_fallen        = 1U;
            }
            else
            {
                /* ---- Cascade control ---- */
                g_loop_counter++;

                /* Average wheel position (degrees) and RPM from both wheels */
                float avg_pos = (Encoder_Left_GetAngleDeg() + Encoder_Right_GetAngleDeg()) * 0.5f;
                float avg_rpm = (Encoder_Left_GetRPM()      + Encoder_Right_GetRPM())      * 0.5f;

                /* 1. Position loop: outer-most (25Hz = every 40ms) */
                if ((g_loop_counter % POSITION_LOOP_DIVIDER) == 0U)
                {
                    float pos_dt = CONTROL_LOOP_DT_SEC * (float)POSITION_LOOP_DIVIDER;
                    g_target_rpm = PID_PositionUpdate(&pid_position,
                                                      avg_pos,
                                                      g_target_position,
                                                      pos_dt);
                }

                /* 2. Velocity loop: middle (50Hz = every 20ms) */
                if ((g_loop_counter % VELOCITY_LOOP_DIVIDER) == 0U)
                {
                    float vel_dt = CONTROL_LOOP_DT_SEC * (float)VELOCITY_LOOP_DIVIDER;
                    g_target_angle = PID_VelocityUpdate(&pid_velocity,
                                                        avg_rpm,
                                                        g_target_rpm,
                                                        vel_dt);
                }

                /* 3. Balance loop: inner-most (200Hz = every 5ms) */
                float balance_target = MECHANICAL_ZERO_ANGLE + g_target_angle;
                g_balance_target     = balance_target;

                float output = PID_BalanceUpdate(&pid_balance,
                                                 pitch,
                                                 balance_target,
                                                 imu.gy,
                                                 CONTROL_LOOP_DT_SEC);

                /* Compensate motor static deadzone (continuous) */
                g_motor_output = Balance_CompensateDeadzone(output);
                Motor_SetSpeed(g_motor_output, g_motor_output);
            }
        }

        /* ---- Telemetry 50Hz: only pack data here, transmit from thread_0 ---- */
        g_telemetry_counter++;
        if ((g_telemetry_counter % 4U) == 0U)
        {
            robot_data.header = 0xFC;
            robot_data.length = 24U;  /* 12 int16_t values */

            robot_data.accel[0] = sat_i16(imu.ax * 1000.0f);
            robot_data.accel[1] = sat_i16(imu.ay * 1000.0f);
            robot_data.accel[2] = sat_i16(imu.az * 1000.0f);
            robot_data.gyro[0]  = sat_i16(imu.gx * 100.0f);
            robot_data.gyro[1]  = sat_i16(imu.gy * 100.0f);
            robot_data.gyro[2]  = sat_i16(imu.gz * 100.0f);

            robot_data.pitch_x100     = sat_i16(imu.kalman_pitch * 100.0f);
            robot_data.target_x100    = sat_i16(g_balance_target * 100.0f);
            robot_data.left_rpm_x100  = sat_i16(Encoder_Left_GetRPM()  * 100.0f);
            robot_data.right_rpm_x100 = sat_i16(Encoder_Right_GetRPM() * 100.0f);
            robot_data.position_x10   = sat_i16(((Encoder_Left_GetAngleDeg() +
                                                  Encoder_Right_GetAngleDeg()) * 0.5f) * 10.0f);
            robot_data.motor_output   = g_motor_output;

            robot_data.end = 0xFF;

            g_telemetry_pending = 1U;
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

    USB_Reset_DP();

    /* USB Device initialize */
    MX_USB_DEVICE_Init();

    /* Peripheral initialisations */
    led_init();
    debug_uart_init();
    TRACE_INFO("STM32F1 Application started\n");

    /* Hand control to the ThreadX kernel - never returns */
    tx_kernel_enter();

    /* Infinite loop fallback */
    while (1)
    {
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

    /* Create Thread 0 (Setup + telemetry) */
    tx_byte_allocate(&byte_pool_0, (VOID **)&pointer, THREAD0_STACK_SIZE, TX_NO_WAIT);
    tx_thread_create(&thread_0, "thread 0", thread_0_entry, 0UL,
                     pointer, THREAD0_STACK_SIZE,
                     1U, 1U, TX_NO_TIME_SLICE, TX_AUTO_START);

    /* Create Thread 1 (Housekeeping) */
    tx_byte_allocate(&byte_pool_0, (VOID **)&pointer, THREAD1_STACK_SIZE, TX_NO_WAIT);
    tx_thread_create(&thread_1, "thread 1", thread_1_entry, 1UL,
                     pointer, THREAD1_STACK_SIZE,
                     16U, 16U, 4U, TX_AUTO_START);

    TRACE_INFO("OS init done.\n");
}

/**
 * @brief  Thread 0 body - robot setup, then USB telemetry transmit.
 *         tx_thread_sleep(1) = 1 tick; check TX_TIMER_TICKS_PER_SECOND
 *         (100 -> 10 ms, enough for 50Hz telemetry).
 * @param  thread_input  Thread parameter.
 */
static void thread_0_entry(ULONG thread_input)
{
    TX_INTERRUPT_SAVE_AREA
    usb_package_t pkt;

    (void)thread_input;

    /* Initialize IMU, calibrate and start 200Hz balancing timer */
    Robot_Setup();

    while (1)
    {
        if (g_telemetry_pending && hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED)
        {
            /* Copy snapshot atomically so the ISR cannot overwrite it mid-copy */
            TX_DISABLE
            pkt = robot_data;
            g_telemetry_pending = 0U;
            TX_RESTORE

            (void)CDC_Transmit_FS((uint8_t *)&pkt, (uint16_t)(pkt.length + 3U));
        }

        tx_thread_sleep(1U);
    }
}

/**
 * @brief  Thread 1 body - background housekeeping tasks.
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
void Error_Handler(void)
{
    while (1)
    {
    }
}

/***************************************************************************************************************
**                                                End of file
***************************************************************************************************************/