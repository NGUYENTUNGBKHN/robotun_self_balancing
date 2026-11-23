/*
****************************************************************************************************************
 * @file       MPU6050_driver.c
 * @brief      MPU6050 I2C driver for STM32F1 (PB6=SCL, PB7=SDA), Repeated Start, polling
 * @date       2026/11/14
 * @author     [Gentantun] (nguyenthanhtung8196@gmail.com)
 * @details    I2C1 on STM32F103: PB6=SCL, PB7=SDA. No DMA, no interrupt needed.
 *             HAL_I2C_Mem_Read generates:
 *             START | ADDR+W | REG | REPEATED START | ADDR+R | DATA... | NACK | STOP
 * @ref        MPU6050 Register Map
 * @copyright  Copyright (c) 2026 RoboTun
****************************************************************************************************************
*/
/***************************************************************************************************************
**                                                   INCLUDES
***************************************************************************************************************/
#include <stdint.h>
#include "common.h"
#include "stm32f1xx_hal.h"
#include "MPU6050_driver.h"

/***************************************************************************************************************
**                                        INTERNAL VARIABLE DEFINITIONS
***************************************************************************************************************/
static I2C_HandleTypeDef hi2c1;

/* Initial configuration: {register, value} */
static const struct { uint8_t reg; uint8_t val; } s_cfg[] =
{
    { MPU6050_SMPLRT_DIV,   0x04U                  },   /* 1kHz / (1 + 4) = 200Hz */
    { MPU6050_CONFIG,       MPU6050_DLPF_CFG_94HZ  },   /* DLPF 94Hz accel / 98Hz gyro */
    { MPU6050_GYRO_CONFIG,  MPU6050_GYRO_FS_500DPS },   /* +/- 500 deg/s */
    { MPU6050_ACCEL_CONFIG, MPU6050_ACCEL_FS_2G    },   /* +/- 2g */
};

/***************************************************************************************************************
**                                         INTERNAL FUNCTION PROTOTYPES
***************************************************************************************************************/
static void          MPU6050_I2C_Config_Pin(void);
static code_status_t MPU6050_I2C_Init(I2C_HandleTypeDef *hi2c);

/***************************************************************************************************************
**                                             FUNCTION DEFINITIONS
***************************************************************************************************************/

/*
****************************************************************************************************************
 *                  MPU6050_I2C_Init
 * @brief      Initialize I2C1 peripheral (PB6=SCL, PB7=SDA)
 * @param      hi2c            :  I2C handle
 * @return     code_status_t
****************************************************************************************************************
*/
static code_status_t MPU6050_I2C_Init(I2C_HandleTypeDef *hi2c)
{
    hi2c->Instance             = MPU6050_INSTANCE;
    hi2c->Init.ClockSpeed      = MPU6050_CLOCK_SPEED;
    hi2c->Init.DutyCycle       = MPU6050_DUTY_CYCLE;
    hi2c->Init.OwnAddress1     = 0x00U;
    hi2c->Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c->Init.OwnAddress2     = 0x00U;
    hi2c->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c->Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(hi2c) != HAL_OK)
    {
        return ACE_ERROR;
    }
    return ACE_SUCCESS;
}

/*
****************************************************************************************************************
 *              MPU6050_I2C_Config_Pin
 * @brief      I2C1 GPIO and clock configuration
****************************************************************************************************************
*/
static void MPU6050_I2C_Config_Pin(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable clocks */
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* Configure PB6 (SCL) and PB7 (SDA) */
    GPIO_InitStruct.Pin   = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_OD;            /* Alternate function open-drain */
    GPIO_InitStruct.Pull  = GPIO_PULLUP;                /* External pull-ups required, internal as backup */
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/*
****************************************************************************************************************
 *                  MPU6050_WriteReg
 * @brief               Write a single register
 * @param      self     : Driver instance
 * @param      reg      : Register address
 * @param      data     : Data to write
 * @return     code_status_t
****************************************************************************************************************
*/
code_status_t MPU6050_WriteReg(MPU6050_driver_t *self, uint8_t reg, uint8_t data)
{
    /* START | ADDR+W | REG | DATA | STOP */
    if (HAL_I2C_Mem_Write(self->handle, MPU6050_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                          &data, 1, MPU6050_I2C_TIMEOUT_MS) != HAL_OK)
    {
        return ACE_ERROR;
    }
    return ACE_SUCCESS;
}

/*
****************************************************************************************************************
 *                  MPU6050_ReadRegs
 * @brief               Read multiple registers (repeated start)
 * @param      self    : Driver instance
 * @param      reg     : Starting register address
 * @param      data    : Buffer to store read data
 * @param      len     : Number of bytes to read
 * @return     code_status_t
****************************************************************************************************************
*/
code_status_t MPU6050_ReadRegs(MPU6050_driver_t *self, uint8_t reg, uint8_t *data, uint16_t len)
{
    /* START | ADDR+W | REG | RESTART | ADDR+R | DATA... | STOP */
    if (HAL_I2C_Mem_Read(self->handle, MPU6050_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                         data, len, MPU6050_I2C_TIMEOUT_MS) != HAL_OK)
    {
        return ACE_ERROR;
    }
    return ACE_SUCCESS;
}

/*
****************************************************************************************************************
 *                  MPU6050_ReadReg
 * @brief               Read a single register (repeated start)
****************************************************************************************************************
*/
code_status_t MPU6050_ReadReg(MPU6050_driver_t *self, uint8_t reg, uint8_t *data)
{
    return MPU6050_ReadRegs(self, reg, data, 1);
}

/*
****************************************************************************************************************
 *                      MPU6050_WhoAmI
 * @brief               Read WHO_AM_I register (should return 0x68 for MPU6050)
****************************************************************************************************************
*/
code_status_t MPU6050_WhoAmI(MPU6050_driver_t *self, uint8_t *id)
{
    return MPU6050_ReadReg(self, MPU6050_WHO_AM_I_REG, id);
}

/*
****************************************************************************************************************
 *                      MPU6050_Init
 * @brief               Initialize I2C and configure MPU6050 sensor
 * @return     code_status_t
****************************************************************************************************************
*/
code_status_t MPU6050_Init(MPU6050_driver_t *self)
{
    uint8_t who_am_i = 0;
    uint32_t i;

    /* Initialize GPIO and I2C peripheral */
    MPU6050_I2C_Config_Pin();
    if (MPU6050_I2C_Init(self->handle) != ACE_SUCCESS)
    {
        return ACE_ERROR;
    }

    /* Wait for sensor to power up */
    HAL_Delay(100);

    /* Verify WHO_AM_I register (0x68 for MPU6050) */
    if (MPU6050_WhoAmI(self, &who_am_i) != ACE_SUCCESS)
    {
        return ACE_ERROR;
    }
    // TRACE_INFO("who_am_i = %x\n", who_am_i);
    if ((who_am_i != MPU6050_WHO_AM_I_VAL) && (who_am_i != MPU6050_WHO_AM_I_VAL_FAKE))
    {
        return ACE_ERROR;
    }
    // TRACE_INFO("pass\n");
    /* Reset device to ensure clean state (reset value of PWR_MGMT_1 = sleep) */
    if (MPU6050_WriteReg(self, MPU6050_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET) != ACE_SUCCESS)
    {
        return ACE_ERROR;
    }
    HAL_Delay(100);

    /* Reset analog/digital signal paths */
    if (MPU6050_WriteReg(self, MPU6050_SIGNAL_PATH_RESET, 0x07U) != ACE_SUCCESS)
    {
        return ACE_ERROR;
    }
    HAL_Delay(100);

    /* Wake up, clock source = PLL with X-axis gyro reference */
    if (MPU6050_WriteReg(self, MPU6050_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_PLL_XGYRO) != ACE_SUCCESS)
    {
        return ACE_ERROR;
    }
    HAL_Delay(10);

    /* Sample rate, DLPF, gyro/accel full scale */
    for (i = 0; i < (sizeof(s_cfg) / sizeof(s_cfg[0])); i++)
    {
        if (MPU6050_WriteReg(self, s_cfg[i].reg, s_cfg[i].val) != ACE_SUCCESS)
        {
            return ACE_ERROR;
        }
    }

    return ACE_SUCCESS;
}

/*
****************************************************************************************************************
 *                          MPU6050_ReadRaw
 * @brief                   Read accel + temp + gyro in one 14-byte burst (repeated start)
 * @param      raw      : Output structure
****************************************************************************************************************
*/
code_status_t MPU6050_ReadRaw(MPU6050_driver_t *self, MPU6050_raw_t *raw)
{
    uint8_t buf[MPU6050_BURST_LEN];

    if (MPU6050_ReadRegs(self, MPU6050_ACCEL_XOUT_H_REG, buf, MPU6050_BURST_LEN) != ACE_SUCCESS)
    {
        return ACE_ERROR;
    }

    /* Big-endian parse */
    raw->accel[0] = (int16_t)(((uint16_t)buf[0]  << 8) | buf[1]);
    raw->accel[1] = (int16_t)(((uint16_t)buf[2]  << 8) | buf[3]);
    raw->accel[2] = (int16_t)(((uint16_t)buf[4]  << 8) | buf[5]);
    raw->temp     = (int16_t)(((uint16_t)buf[6]  << 8) | buf[7]);
    raw->gyro[0]  = (int16_t)(((uint16_t)buf[8]  << 8) | buf[9]);
    raw->gyro[1]  = (int16_t)(((uint16_t)buf[10] << 8) | buf[11]);
    raw->gyro[2]  = (int16_t)(((uint16_t)buf[12] << 8) | buf[13]);

    return ACE_SUCCESS;
}

/*
****************************************************************************************************************
 *                          MPU6050_ReadRawData
 * @brief                   Read raw accelerometer and gyroscope data
 * @param      accel    : Array of 3 int16_t for accel X,Y,Z
 * @param      gyro     : Array of 3 int16_t for gyro X,Y,Z
****************************************************************************************************************
*/
code_status_t MPU6050_ReadRawData(MPU6050_driver_t *self, int16_t *accel, int16_t *gyro)
{
    MPU6050_raw_t raw;

    if (MPU6050_ReadRaw(self, &raw) != ACE_SUCCESS)
    {
        return ACE_ERROR;
    }

    for (uint32_t i = 0; i < 3; i++)
    {
        accel[i] = raw.accel[i];
        gyro[i]  = raw.gyro[i];
    }
    return ACE_SUCCESS;
}

/*
****************************************************************************************************************
 *                          MPU6050_Recover
 * @brief      Re-initialize I2C peripheral after an error (NACK, bus error, stuck BUSY flag).
 *             Sensor registers are kept, no need to call init() again.
****************************************************************************************************************
*/
code_status_t MPU6050_Recover(MPU6050_driver_t *self)
{
    (void)HAL_I2C_DeInit(self->handle);

    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();

    return MPU6050_I2C_Init(self->handle);
}

/*
****************************************************************************************************************
 *                      MPU6050_driver_create
 * @brief      Create MPU6050 driver instance
 * @return     MPU6050_driver_t*
****************************************************************************************************************
*/
MPU6050_driver_t *MPU6050_driver_create(void)
{
    static MPU6050_driver_t driver;

    driver.handle      = &hi2c1;                /* I2C1 handle */
    driver.init        = MPU6050_Init;
    driver.writeReg    = MPU6050_WriteReg;
    driver.readReg     = MPU6050_ReadReg;
    driver.readRegs    = MPU6050_ReadRegs;
    driver.who_am_i    = MPU6050_WhoAmI;
    driver.readRawData = MPU6050_ReadRawData;
    driver.readRaw     = MPU6050_ReadRaw;
    driver.recover     = MPU6050_Recover;

    return &driver;
}

/***************************************************************************************************************
**                                                End of file
****************************************************************************************************************
*/
