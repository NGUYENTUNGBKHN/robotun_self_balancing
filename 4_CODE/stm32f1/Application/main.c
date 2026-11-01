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
#include "MPU6500_driver.h"
/***************************************************************************************************************
**                                          INTERNAL MACRO DEFINITIONS
***************************************************************************************************************/
/* ThreadX resource sizing */
#define DEMO_STACK_SIZE         1024U       /* Stack size per thread (bytes) */
#define DEMO_BYTE_POOL_SIZE     9120U       /* Total byte pool for all stacks */

/***************************************************************************************************************
**                                        INTERNAL VARIABLE DEFINITIONS
***************************************************************************************************************/

static ULONG        thread_0_counter = 0U;
static ULONG        thread_1_counter = 0U;

static TX_BYTE_POOL byte_pool_0;
static TX_THREAD    thread_0;
static TX_THREAD    thread_1;
static UCHAR        memory_area[DEMO_BYTE_POOL_SIZE];

/***************************************************************************************************************
**                                         INTERNAL FUNCTION PROTOTYPES
***************************************************************************************************************/
static void SystemClock_Config(void);
static void Error_Handler(void);

/* ThreadX thread entry functions */
static void thread_0_entry(ULONG thread_input);
static void thread_1_entry(ULONG thread_input);

static MPU6500_driver_t *mpu;

/***************************************************************************************************************
**                                             FUNCTION DEFINITIONS
***************************************************************************************************************/

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
    int16_t accel[3], gyro[3];
    /* Initialise HAL */
    HAL_Init();

    /* Configure system clock: HSE 8 MHz -> PLL -> 72 MHz SYSCLK */
    SystemClock_Config();

    /* Peripheral initialisations */
    led_init();
    debug_uart_init();
    TRACE_INFO("STM32F1 Application started\n");
    mpu = MPU6500_driver_create();
    if (mpu->init(mpu) == HAL_OK) 
    {
        mpu->readRawData(mpu, accel, gyro);
    }

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
    int16_t accel[3], gyro[3];
    (void)thread_input;

    while (1)
    {
        thread_0_counter++;
        mpu->readRawData(mpu, accel, gyro);
        TRACE_INFO("accel : (%x, %x, %x) \n", accel[0], accel[1], accel[2]);
        TRACE_INFO("gyro  : (%x, %x, %x) \n", gyro[0], gyro[1], gyro[2]);
        

        /* Sleep 500 ThreadX ticks (500 ms @ 100 Hz tick) */
        tx_thread_sleep(50U);
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
        thread_1_counter++;

        /* Background tasks go here */
        tx_thread_sleep(10U);
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
