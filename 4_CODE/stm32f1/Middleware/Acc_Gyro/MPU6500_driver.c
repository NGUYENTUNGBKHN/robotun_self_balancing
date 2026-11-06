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
#include "common.h"
#include "stm32f1xx_hal.h"
#include "MPU6500_driver.h"

/***************************************************************************************************************
**                                         EXTERNAL FUNCTION PROTOTYPES
***************************************************************************************************************/

/***************************************************************************************************************
**                                        EXTERNAL VARIABLE DECLARATIONS
***************************************************************************************************************/

/***************************************************************************************************************
**                                          INTERNAL MACRO DEFINITIONS
***************************************************************************************************************/


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
static void MPU6500_I2C_Config_Pin();
static code_status_t MPU6500_I2C_Init(I2C_HandleTypeDef *hi2c);

/***************************************************************************************************************
**                                             FUNCTION DEFINITIONS
***************************************************************************************************************/

/*
****************************************************************************************************************
 *                  MPU6500_I2C_Init
 * @brief      Initialize I2C1 peripheral (PB6=SCL, PB7=SDA)
 * @param      hi2c            :  I2C handle
 * @return     code_status_t
 * 
****************************************************************************************************************
*/
static code_status_t MPU6500_I2C_Init(I2C_HandleTypeDef *hi2c)
{
    hi2c->Instance             = MPU6500_INSTANCE;          /* Instance */
    hi2c->Init.ClockSpeed      = MPU6500_CLOCK_SPEED;       /* Clock Speed 400kHz */
    hi2c->Init.DutyCycle       = MPU6500_DUTY_CYCLE;        /* Duty cycle divide 2 */
    hi2c->Init.OwnAddress1     = 0x00U;
    hi2c->Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;   /* Mode 7-bit */
    hi2c->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;   /* Dual address disable */
    hi2c->Init.OwnAddress2     = 0x00U;
    hi2c->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;   /* Generalcall disable */
    hi2c->Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;     /* nostrethc disable */

    /* Initialize I2C1  */
    if (HAL_I2C_Init(&hi2c) != HAL_OK)
    {
        return ACE_ERROR;
    }
    return ACE_SUCCESS;
}

/*
****************************************************************************************************************
 *              MPU6500_I2C_Config_Pin
 * @brief      I2C MSP Initialization (GPIO, Clock, NVIC)
 *
****************************************************************************************************************
*/
static void MPU6500_I2C_Config_Pin()
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

/*
****************************************************************************************************************
 *                  MPU6500_WriteReg
 * @brief               Write a single register on MPU6500
 * @param      self     : Driver instance
 * @param      reg      : Register address
 * @param      data     : Data to write
 * @return     code_status_t
 *             ACE_ERROR : Error
 *             ACE_SUCCESS : Success
****************************************************************************************************************
*/
code_status_t MPU6500_WriteReg(MPU6500_driver_t *self, uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = { reg, data };

    /* I2C transmit data */
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(self->handle, MPU6500_I2C_ADDR, 
                                                buf, 2, MPU6500_I2C_TIMEOUT_MS);
    if (status != HAL_OK)
    {
        /* Return error */
        return ACE_ERROR;
    }
    /* Return success */
    return ACE_SUCCESS;
}

/*
****************************************************************************************************************
 *                  MPU6500_ReadReg
 * @brief           Read a single register from MPU6500
 * @param      self    : Driver instance
 * @param      reg     : Register address to read from
 * @param      data    : Pointer to store read data
 * @return     code_status_t
 *              ACE_ERROR : Error    
 *              ACE_SUCCESS : Success
****************************************************************************************************************
*/
code_status_t MPU6500_ReadReg(MPU6500_driver_t *self, uint8_t reg, uint8_t *data)
{
    HAL_StatusTypeDef status;

    /* I2C Transmit data */
    status = HAL_I2C_Master_Transmit(self->handle, MPU6500_I2C_ADDR, &reg, 1, MPU6500_I2C_TIMEOUT_MS);
    if (status != HAL_OK)
    {
        return ACE_ERROR;
    }

    /* I2C Receive data */
    status = HAL_I2C_Master_Receive(self->handle, MPU6500_I2C_ADDR, data, 1, MPU6500_I2C_TIMEOUT_MS);
    if (status != HAL_OK)
    {
        return ACE_ERROR;
    }
    return ACE_SUCCESS;
}

/*
****************************************************************************************************************
 *                      MPU6500_ReadRegs
 * @brief               Read multiple registers from MPU6500
 * @param      self    : Driver instance
 * @param      reg     : Starting register address
 * @param      data    : Buffer to store read data
 * @param      len     : Number of bytes to read
 * @return     code_status_t
 *              ACE_ERROR : Error    
 *              ACE_SUCCESS : Success
****************************************************************************************************************
*/
code_status_t MPU6500_ReadRegs(MPU6500_driver_t *self, uint8_t reg, uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef status;

    /* Send register address */
    status = HAL_I2C_Master_Transmit(self->handle, MPU6500_I2C_ADDR, &reg, 1, MPU6500_I2C_TIMEOUT_MS);
    if (status != HAL_OK)
    {
        return ACE_ERROR;
    }

    /* Read data */
    status = HAL_I2C_Master_Receive(self->handle, MPU6500_I2C_ADDR, data, len, MPU6500_I2C_TIMEOUT_MS);
    if (status != HAL_OK)
    {
        return ACE_ERROR;
    }
    return ACE_SUCCESS;
}

/*
****************************************************************************************************************
 *                      MPU6500_WhoAmI
 * @brief               Read WHO_AM_I register (should return 0x70 for MPU6500)
 * @param      self     : Driver instance
 * @param      id       : Pointer to store WHO_AM_I value
 * @return     code_status_t
 *              ACE_ERROR : Error     
 *              ACE_SUCCESS : Success
****************************************************************************************************************
*/
code_status_t MPU6500_WhoAmI(MPU6500_driver_t *self, uint8_t *id)
{
    /* Read WHO_AM_I register */
    return MPU6500_ReadReg(self, MPU6500_WHO_AM_I_REG, id);
}

/*
****************************************************************************************************************
 *                      MPU6500_Init
 * @brief               Initialize MPU6500 sensor
 * @param      self     : Driver instance
 * @return     code_status_t
 *              ACE_ERROR : Error    
 *              ACE_SUCCESS : Success
****************************************************************************************************************
*/
code_status_t MPU6500_Init(MPU6500_driver_t *self)
{
    uint8_t who_am_i = 0;
    HAL_StatusTypeDef status;

    /* Initialize I2C peripheral and GPIO */
    MPU6500_I2C_Config_Pin();
    MPU6500_I2C_Init(self->handle);

    /* Wait for sensor to power up */
    HAL_Delay(100);

    /* Verify WHO_AM_I register (0x70 for MPU6500) */
    status = (HAL_StatusTypeDef)MPU6500_WhoAmI(self, &who_am_i);
    if (status != HAL_OK || who_am_i != MPU6500_WHO_AM_I_VAL)
    {
        return ACE_ERROR;
    }

    /* Reset device to ensure clean state */
    status = (HAL_StatusTypeDef)MPU6500_WriteReg(self, MPU6500_PWR_MGMT_1, 0x80U);
    if (status != HAL_OK)
    {
        return ACE_ERROR;
    }
    HAL_Delay(100);
    
    /* Wake up MPU6500 and select Auto Clock Source (PLL with Gyro reference: 0x01) */
    status = (HAL_StatusTypeDef)MPU6500_WriteReg(self, MPU6500_PWR_MGMT_1, 0x01U);
    if (status != HAL_OK)
    {
        return ACE_ERROR;
    }
    HAL_Delay(10);

    /* Set sample rate divider (1kHz / (1 + 4) = 200Hz - optimal for balancing loop) */
    status = (HAL_StatusTypeDef)MPU6500_WriteReg(self, MPU6500_SMPLRT_DIV, 0x04U);
    if (status != HAL_OK)
    {
        return ACE_ERROR;
    }

    /* Configure Gyro/Temp DLPF - DLPF_CFG = 2 (Bandwidth 92Hz, Delay 3.9ms) */
    status = (HAL_StatusTypeDef)MPU6500_WriteReg(self, MPU6500_CONFIG, 0x02U);
    if (status != HAL_OK)
    {
        return ACE_ERROR;
    }

    /* Configure Gyro full scale range: +/- 500 deg/s (FS_SEL = 1 -> 0x08) */
    status = (HAL_StatusTypeDef)MPU6500_WriteReg(self, MPU6500_GYRO_CONFIG, 0x08U);
    if (status != HAL_OK)
    {
        return ACE_ERROR;
    }

    /* Configure Accel full scale range: +/- 2g (AFS_SEL = 0 -> 0x00) */
    status = (HAL_StatusTypeDef)MPU6500_WriteReg(self, MPU6500_ACCEL_CONFIG, 0x00U);
    if (status != HAL_OK)
    {
        return ACE_ERROR;
    }

    /* Configure Accel DLPF - A_DLPF_CFG = 2 (Bandwidth 92Hz, Delay 3.8ms, ODR 1kHz) */
    status = (HAL_StatusTypeDef)MPU6500_WriteReg(self, MPU6500_ACCEL_CONFIG_2, 0x02U);
    if (status != HAL_OK)
    {
        return ACE_ERROR;
    }

    return ACE_SUCCESS;
}

/*
****************************************************************************************************************
 *                          MPU6500_ReadRawData
 * @brief                   Read raw accelerometer and gyroscope data
 * @param      self     : Driver instance
 * @param      accel    : Array of 3 int16_t for accel X,Y,Z
 * @param      gyro     : Array of 3 int16_t for gyro X,Y,Z
 * @return     code_status_t
 *              ACE_ERROR : Error    
 *              ACE_SUCCESS : Success
****************************************************************************************************************
*/
code_status_t MPU6500_ReadRawData(MPU6500_driver_t *self, int16_t *accel, int16_t *gyro)
{
    uint8_t buf[14];
    HAL_StatusTypeDef status;

    /* Read 14 bytes starting from ACCEL_XOUT_H (0x3B) */
    status = (HAL_StatusTypeDef)MPU6500_ReadRegs(self, MPU6500_ACCEL_XOUT_H_REG, buf, 14);
    if (status != HAL_OK)
    {
        return ACE_ERROR;
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

    return ACE_SUCCESS;
}

/*
****************************************************************************************************************
 *                      MPU6500_driver_create
 * @brief      Create MPU6500 driver instance
 * @return     MPU6500_driver_t*
****************************************************************************************************************
*/
MPU6500_driver_t *MPU6500_driver_create(void)
{
    static MPU6500_driver_t driver;

    driver.handle = &hi2c1;                     /* I2C1 handle  */
    driver.init   = MPU6500_Init;               /* Initialize function */
    driver.writeReg = MPU6500_WriteReg;         /* Write function */
    driver.readReg  = MPU6500_ReadReg;          /* Read function */
    driver.who_am_i = MPU6500_WhoAmI;           /* Read WHO_AM_I register */
    driver.readRawData = MPU6500_ReadRawData;   /* Read raw data */

    /* Return driver */
    return &driver;
}

/***************************************************************************************************************
**                                                End of file
****************************************************************************************************************
*/