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
#include <string.h>
#include "boot_main.h"
#include "debug_uart.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
/***************************************************************************************************************
**                                         INTERNAL FUNCTION PROTOTYPES
***************************************************************************************************************/
static void SystemClock_Config(void);
static void USB_Reset_DP(void);

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
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_Delay(50);
}

/**
 * @brief  Bootloader main entry point.
 * @return int Status code (never returns normally).
 */
int boot_main(void)
{
    char test[] = "USB CDC test message\r\n";
    /* HAL initialize */
    HAL_Init();

    /* Clock configuration: 72 MHz SYSCLK */
    SystemClock_Config();

    /* Debug UART initialize (USART1 PA9 @ 115200 baud) */
    debug_uart_init();

    TRACE_INFO("STM32F1 Bootloader started\n");

    /* Force USB re-enumeration on PC */
    USB_Reset_DP();

    /* USB Device initialize */
    MX_USB_DEVICE_Init();

    while (1)
    {
        /* Transmit only when USB is connected and configured by PC */
        if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED)
        {
            CDC_Transmit_FS((uint8_t *)test, strlen(test));
        }
        HAL_Delay(1000);
    }
}


/**
 * @brief  System clock configuration for STM32F103xB:
 *         SYSCLK = 72 MHz (HSE 8 MHz x 9)
 */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
    PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
