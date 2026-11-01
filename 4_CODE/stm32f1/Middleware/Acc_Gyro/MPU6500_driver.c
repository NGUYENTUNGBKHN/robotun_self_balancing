/*
****************************************************************************************************************
 * @file       MPU6500_driver.c
 * @brief      MPU6500 I2C driver for STM32F1 (PB6=SCL, PB7=SDA)
 * @date       2026/10/29
 * @author     [Gentantun] (nguyenthanhtung8196@gmail.com)
 * @details    I2C1 on STM32F103: PB6=SCL, PB7=SDA
 * @ref        MPU6500 Register Map
 * @copyright  Copyright (c) 2026 RoboTun
****************************************************************************************************************
*/
/***************************************************************************************************************
**                                                   INCLUDES
***************************************************************************************************************/
#include <stdint.h>
#include <string.h>
#include "common.h"
#include "MPU6500_driver.h"
#include "stm32f1xx_hal.h"

/***************************************************************************************************************
**                                         EXTERNAL FUNCTION PROTOTYPES
***************************************************************************************************************/

/***************************************************************************************************************
**                                        EXTERNAL VARIABLE DECLARATIONS
***************************************************************************************************************/

/***************************************************************************************************************
**                                          INTERNAL MACRO DEFINITIONS
***************************************************************************************************************/
#define MPU6500_I2C_ADDR          (0x68U << 1)  /* 7-bit address 0x68 shifted for HAL */
#define MPU6500_WHO_AM_I_REG      0x75U
#define MPU6500_PWR_MGMT_1_REG    0x6BU
#define MPU6500_SMPLRT_DIV_REG    0x19U
#define MPU6500_CONFIG_REG        0x1AU
#define MPU6500_GYRO_CONFIG_REG   0x1BU
#define MPU6500_ACCEL_CONFIG_REG  0x1CU
#define MPU6500_ACCEL_XOUT_H_REG  0x3BU
#define MPU6500_I2C_TIMEOUT_MS    100U

/***************************************************************************************************************
**                                         COMMON VARIABLE DEFINITIONS
***************************************************************************************************************/

/***************************************************************************************************************
**                                        INTERNAL VARIABLE DEFINITIONS
***************************************************************************************************************/
static I2C_HandleTypeDef hi2c1;

/***************************************************************************************************************
**                                         INTERNAL FUNCTION PROTOTYPES
***************************************************************************************************************/
static void MPU6500_I2C_MspInit(I2C_HandleTypeDef *hi2c);
static code_status_t MPU6500_I2C_Init(void);

/***************************************************************************************************************
**                                             FUNCTION DEFINITIONS
***************************************************************************************************************/

/**
 * @brief  Initialize I2C1 peripheral (PB6=SCL, PB7=SDA)
 * @note   Standard mode @ 100kHz
 */
static code_status_t MPU6500_I2C_Init(void)
{
    hi2c1.Instance             = I2C1;
    hi2c1.Init.ClockSpeed      = 100000U;               /* 100 kHz standard mode */
    hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1     = 0x00U;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2     = 0x00U;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        TRACE_INFO("Error\n");
        return ACE_ERROR;
    }
    return ACE_SUCCESS;
}

/**
 * @brief  I2C MSP Initialization (GPIO, Clock, NVIC)
 * @param  hi2c: I2C handle
 */
static void MPU6500_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // if (hi2c->Instance == I2C1)
    {
        /* Enable clocks */
        __HAL_RCC_I2C1_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /* Configure PB6 (SCL) and PB7 (SDA) */
        GPIO_InitStruct.Pin       = GPIO_PIN_6;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;      /* Alternate function open-drain */
        GPIO_InitStruct.Pull      = GPIO_PULLUP;          /* External pull-ups required, internal as backup */
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;

        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        GPIO_InitStruct.Pin         = GPIO_PIN_7;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        // /* NVIC configuration for I2C1 event interrupt */
        // HAL_NVIC_SetPriority(I2C1_EV_IRQn, 5, 0);
        // HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);

        // /* NVIC configuration for I2C1 error interrupt */
        // HAL_NVIC_SetPriority(I2C1_ER_IRQn, 5, 0);
        // HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
    }
}

/**
 * @brief  Write a single register on MPU6500
 * @param  self: Driver instance
 * @param  reg: Register address
 * @param  data: Data to write
 * @return HAL status
 */
uint32_t MPU6500_WriteReg(MPU6500_driver_t *self, uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = { reg, data };
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, MPU6500_I2C_ADDR, buf, 2, MPU6500_I2C_TIMEOUT_MS);
    return (uint32_t)status;
}

/**
 * @brief  Read a single register from MPU6500
 * @param  self: Driver instance
 * @param  reg: Register address to read from
 * @param  data: Pointer to store read data
 * @return HAL status
 */
uint32_t MPU6500_ReadReg(MPU6500_driver_t *self, uint8_t reg, uint8_t *data)
{
    HAL_StatusTypeDef status;

    /* Send register address */
    status = HAL_I2C_Master_Transmit(&hi2c1, MPU6500_I2C_ADDR, &reg, 1, MPU6500_I2C_TIMEOUT_MS);
    if (status != HAL_OK)
    {
        return (uint32_t)status;
    }

    /* Read data */
    status = HAL_I2C_Master_Receive(&hi2c1, MPU6500_I2C_ADDR, data, 1, MPU6500_I2C_TIMEOUT_MS);
    return (uint32_t)status;
}

/**
 * @brief  Read multiple registers from MPU6500
 * @param  self: Driver instance
 * @param  reg: Starting register address
 * @param  data: Buffer to store read data
 * @param  len: Number of bytes to read
 * @return HAL status
 */
uint32_t MPU6500_ReadRegs(MPU6500_driver_t *self, uint8_t reg, uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef status;

    /* Send register address */
    status = HAL_I2C_Master_Transmit(&hi2c1, MPU6500_I2C_ADDR, &reg, 1, MPU6500_I2C_TIMEOUT_MS);
    if (status != HAL_OK)
    {
        return (uint32_t)status;
    }

    /* Read data */
    status = HAL_I2C_Master_Receive(&hi2c1, MPU6500_I2C_ADDR, data, len, MPU6500_I2C_TIMEOUT_MS);
    return (uint32_t)status;
}

/**
 * @brief  Read WHO_AM_I register (should return 0x68)
 * @param  self: Driver instance
 * @param  id: Pointer to store WHO_AM_I value
 * @return HAL status
 */
uint32_t MPU6500_WhoAmI(MPU6500_driver_t *self, uint8_t *id)
{
    return MPU6500_ReadReg(self, MPU6500_WHO_AM_I_REG, id);
}

/**
 * @brief  Initialize MPU6500 sensor
 * @param  self: Driver instance
 * @return HAL status
 */
uint32_t MPU6500_Init(MPU6500_driver_t *self)
{
    uint8_t who_am_i = 0;
    HAL_StatusTypeDef status;

    /* Initialize I2C peripheral and GPIO */
    MPU6500_I2C_MspInit(&hi2c1);
    MPU6500_I2C_Init();

    /* Wait for sensor to power up */
    HAL_Delay(100);

    /* Verify WHO_AM_I register */
    status = (HAL_StatusTypeDef)MPU6500_WhoAmI(self, &who_am_i);
    if (status != HAL_OK || who_am_i != 0x70U)
    {
        return (uint32_t)HAL_ERROR;
    }
    
    /* Wake up MPU6500 (clear sleep bit) */
    status = (HAL_StatusTypeDef)MPU6500_WriteReg(self, MPU6500_PWR_MGMT_1_REG, 0x00U);
    if (status != HAL_OK)
    {
        return (uint32_t)status;
    }
    HAL_Delay(10);

    /* Set sample rate divider (1kHz / (1 + 9) = 100Hz) */
    status = (HAL_StatusTypeDef)MPU6500_WriteReg(self, MPU6500_SMPLRT_DIV_REG, 0x09U);
    if (status != HAL_OK)
    {
        return (uint32_t)status;
    }

    /* Configure DLPF (Digital Low Pass Filter) - Bandwidth 44Hz */
    status = (HAL_StatusTypeDef)MPU6500_WriteReg(self, MPU6500_CONFIG_REG, 0x03U);
    if (status != HAL_OK)
    {
        return (uint32_t)status;
    }

    /* Configure Gyro full scale range ±250 °/s */
    status = (HAL_StatusTypeDef)MPU6500_WriteReg(self, MPU6500_GYRO_CONFIG_REG, 0x00U);
    if (status != HAL_OK)
    {
        return (uint32_t)status;
    }

    /* Configure Accel full scale range ±2g */
    status = (HAL_StatusTypeDef)MPU6500_WriteReg(self, MPU6500_ACCEL_CONFIG_REG, 0x00U);
    if (status != HAL_OK)
    {
        return (uint32_t)status;
    }

    return (uint32_t)HAL_OK;
}

/**
 * @brief  Read raw accelerometer and gyroscope data
 * @param  self: Driver instance
 * @param  accel: Array of 3 int16_t for accel X,Y,Z
 * @param  gyro: Array of 3 int16_t for gyro X,Y,Z
 * @return HAL status
 */
uint32_t MPU6500_ReadRawData(MPU6500_driver_t *self, int16_t *accel, int16_t *gyro)
{
    uint8_t buf[14];
    HAL_StatusTypeDef status;

    /* Read 14 bytes starting from ACCEL_XOUT_H (0x3B) */
    status = (HAL_StatusTypeDef)MPU6500_ReadRegs(self, MPU6500_ACCEL_XOUT_H_REG, buf, 14);
    if (status != HAL_OK)
    {
        return (uint32_t)status;
    }

    /* Parse accelerometer data (big-endian) */
    accel[0] = (int16_t)((buf[0] << 8) | buf[1]);   /* ACCEL_XOUT */
    accel[1] = (int16_t)((buf[2] << 8) | buf[3]);   /* ACCEL_YOUT */
    accel[2] = (int16_t)((buf[4] << 8) | buf[5]);   /* ACCEL_ZOUT */

    /* Skip temperature (buf[6], buf[7]) */

    /* Parse gyroscope data (big-endian) */
    gyro[0] = (int16_t)((buf[8] << 8) | buf[9]);    /* GYRO_XOUT */
    gyro[1] = (int16_t)((buf[10] << 8) | buf[11]);  /* GYRO_YOUT */
    gyro[2] = (int16_t)((buf[12] << 8) | buf[13]);  /* GYRO_ZOUT */

    return (uint32_t)HAL_OK;
}

/**
 * @brief  Create MPU6500 driver instance
 * @return Pointer to driver instance
 */
MPU6500_driver_t *MPU6500_driver_create(void)
{
    static MPU6500_driver_t driver;

    TRACE_INFO("enter\n");
    driver.handle = &hi2c1;
    driver.init   = MPU6500_Init;
    driver.writeReg = MPU6500_WriteReg;
    driver.readReg  = MPU6500_ReadReg;
    driver.who_am_i = MPU6500_WhoAmI;
    driver.readRawData = MPU6500_ReadRawData;

    return &driver;
}



/***************************************************************************************************************
**                                                End of file
****************************************************************************************************************
*/