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
#include "MPU6500_driver.h"
#include "MPU6050_driver.h"
/***************************************************************************************************************
**                                         INTERNAL FUNCTION PROTOTYPES
***************************************************************************************************************/
static void SystemClock_Config(void);
static void USB_Reset_DP(void);
static void Timer4_Init(void);

MPU6050_driver_t *mpu = NULL;
typedef struct USB_PACKAGE_S
{
    uint8_t header;
    uint8_t length;
    int16_t accel[3];
    int16_t gyro[3];
    uint8_t end;
}usb_package_t;

usb_package_t imu_data;

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

    

    /* Initialize I2C MPU6050 */
    mpu = MPU6050_driver_create();
    mpu->init(mpu);

    /* Force USB re-enumeration on PC */
    USB_Reset_DP();

    /* USB Device initialize */
    MX_USB_DEVICE_Init();

    /* Initialize timer 4 */
    Timer4_Init();

    uint32_t last_tx_tick = 0;

    TRACE_INFO("STM32F1 Bootloader started\n");

    while (1)
    {
        /* Read received bytes from USB RX Ring Buffer */
        while (CDC_Available() > 0)
        {
            int16_t ch = CDC_Read();
            if (ch >= 0)
            {
                uint8_t c = (uint8_t)ch;
                /* Echo received character back to PC */
                CDC_Transmit_FS(&c, 1);
            }
        }

        /* Periodic message every 1 second */
        // if ((HAL_GetTick() - last_tx_tick) >= 1000)
        // {
        //     last_tx_tick = HAL_GetTick();
        //     if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED)
        //     {
        //         CDC_Transmit_FS((uint8_t *)test, strlen(test));
        //     }
        // }
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
static TIM_HandleTypeDef htim4;
static void Timer4_Init(void)
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

    /* Start TIM4 Base Interrupt */
    HAL_TIM_Base_Start_IT(&htim4);
}


void TIM4_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_UPDATE) != RESET)
    {
        if (__HAL_TIM_GET_IT_SOURCE(&htim4, TIM_IT_UPDATE) != RESET)
        {
            __HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);
            if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED)
            {
                mpu->readRawData(mpu, imu_data.accel, imu_data.gyro);

                imu_data.header = 0xFC;
                imu_data.length = 12;
                imu_data.end = 0xFF;
                CDC_Transmit_FS((uint8_t*)&imu_data, imu_data.length + 3);
                // TRACE_INFO("acc = %x, %x, %x\n", raw_accel_gyro[0], raw_accel_gyro[1], raw_accel_gyro[2]);
                // TRACE_INFO("gyro = %x, %x, %x\n", raw_accel_gyro[3], raw_accel_gyro[4], raw_accel_gyro[5]);
            }
        }
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
