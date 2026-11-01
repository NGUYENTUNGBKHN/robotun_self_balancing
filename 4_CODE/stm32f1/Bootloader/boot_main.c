/*
****************************************************************************************************************
 * @file       boot_main.c
 * @brief      Bootloader main entry point and partition dispatcher for STM32F103xB
 * @date       2026/10/25
 * @author     [Gentantun] (nguyenthanhtung8196@gmail.com)
 * @details    Initialises clock system to 72 MHz, configures debug UART on PA9, checks
 *             boot status / DFU flags, and either executes the DFU finite state machine
 *             or branches directly to the application partition at APP_START_ADDRESS.
 * @ref        stm32f7/Bootloader/boot_main.c
 * @copyright  Copyright (c) 2026 RoboTun
****************************************************************************************************************
*/
/***************************************************************************************************************
**                                                   INCLUDES
***************************************************************************************************************/
#include "boot_main.h"
#include "debug_uart.h"

/***************************************************************************************************************
**                                         INTERNAL FUNCTION PROTOTYPES
***************************************************************************************************************/
static void SystemClock_Config(void);
static void Error_Handler(void);
static void jump_to_application(void);
static void jump_to_bootloader(void);

/***************************************************************************************************************
**                                             FUNCTION DEFINITIONS
***************************************************************************************************************/

/**
 * @brief  Bootloader main entry point.
 * @return int Status code (never returns normally).
 */
int boot_main(void)
{
    /* HAL initialize */
    HAL_Init();

    /* Clock configuration: 72 MHz SYSCLK */
    SystemClock_Config();

    /* Debug UART initialize (USART1 PA9 @ 115200 baud) */
    debug_uart_init();

    TRACE_INFO("STM32F1 Bootloader started\n");

    /* Check whether to boot application or stay in bootloader mode */
    /* Verification check: Valid stack pointer at APP_START_ADDRESS */
    uint32_t app_msp = *(volatile uint32_t *)APP_START_ADDRESS;
    if (app_msp != 0xFFFFFFFF)
    {
        TRACE_INFO("Jumping to application at 0x%08X\n", APP_START_ADDRESS);
        jump_to_application();
    }
    else
    {
        TRACE_INFO("No valid application found. Entering DFU mode\n");
        jump_to_bootloader();
    }

    while (1)
    {
        /* Should never be reached */
    }
}

/**
 * @brief  De-initialise core registers, relocate VTOR and jump to application Reset_Handler.
 */
static void jump_to_application(void)
{
    void (*app_reset_handler)(void);

    /* Read application initial MSP and Reset_Handler address */
    uint32_t app_msp_addr           = *(volatile uint32_t *)(APP_START_ADDRESS);
    uint32_t app_reset_handler_addr = *(volatile uint32_t *)(APP_START_ADDRESS + 4U);

    /* Reset core exception registers */
    __set_CONTROL(0x00000000U);
    __set_PRIMASK(0x00000000U);
    __set_BASEPRI(0x00000000U);
    __set_FAULTMASK(0x00000000U);

    app_reset_handler = (void (*)(void))app_reset_handler_addr;

    /* Set Main Stack Pointer to application stack base */
    __set_MSP(app_msp_addr);

    /* Relocate Vector Table Offset Register (VTOR) */
    SCB->VTOR = APP_START_ADDRESS;

    /* Call application reset handler */
    app_reset_handler();
}

/**
 * @brief  Enter bootloader state machine execution loop.
 */
static void jump_to_bootloader(void)
{
    bootloader();
}

/**
 * @brief  System clock configuration for STM32F103xB:
 *         SYSCLK = 72 MHz (HSE 8 MHz x 9)
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
