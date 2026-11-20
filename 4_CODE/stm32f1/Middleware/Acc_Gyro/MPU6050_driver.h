/*
****************************************************************************************************************
 * @file       MPU6050_driver.h
 * @brief      MPU6050 I2C driver header for STM32F1 (Repeated Start, polling)
 * @date       2026/11/14
 * @author     [Gentantun] (nguyenthanhtung8196@gmail.com)
 * @details    I2C1: PB6=SCL, PB7=SDA
 * @copyright  Copyright (c) 2026 RoboTun
****************************************************************************************************************
*/
#ifndef _MPU6050_DRIVER_H_
#define _MPU6050_DRIVER_H_

#ifdef __cplusplus
extern "C"
{
#endif

/***************************************************************************************************************
**                                                   INCLUDES
***************************************************************************************************************/
#include <stdint.h>
#include "common.h"                                         /* code_status_t, ACE_SUCCESS, ACE_ERROR */

/***************************************************************************************************************
**                                            TYPE DEFINITIONS
***************************************************************************************************************/

#define MPU6050_INSTANCE                I2C1                /* use I2C1 */
#define MPU6050_CLOCK_SPEED             400000              /* 400kHz */
#define MPU6050_DUTY_CYCLE              I2C_DUTYCYCLE_2     /* use duty cycle 2 */

#define MPU6050_I2C_ADDR                (0x68U << 1)        /* 7-bit address 0x68 shifted for HAL (AD0=0) */
#define MPU6050_I2C_ADDR_AD0_HIGH       (0x69U << 1)        /* 7-bit address 0x69 shifted for HAL (AD0=1) */
#define MPU6050_WHO_AM_I_VAL            0x68U               /* Default WHO_AM_I value for MPU6050 */
#define MPU6050_WHO_AM_I_VAL_FAKE        0x70U               /* Default WHO_AM_I value for MPU6050 */
#define MPU6050_I2C_TIMEOUT_MS          100U

#define MPU6050_BURST_LEN               14U                 /* ACCEL(6) + TEMP(2) + GYRO(6) */

/* ============================================================================================================
**                                          MPU-6050 REGISTER MAP
** ============================================================================================================ */
/* Accelerometer Offset Cancellation Registers (trim, not in the official register map but widely used) */
#define MPU6050_XA_OFFS_H               0x06U  /* Accel X-axis offset trim high byte */
#define MPU6050_XA_OFFS_L               0x07U  /* Accel X-axis offset trim low byte */
#define MPU6050_YA_OFFS_H               0x08U  /* Accel Y-axis offset trim high byte */
#define MPU6050_YA_OFFS_L               0x09U  /* Accel Y-axis offset trim low byte */
#define MPU6050_ZA_OFFS_H               0x0AU  /* Accel Z-axis offset trim high byte */
#define MPU6050_ZA_OFFS_L               0x0BU  /* Accel Z-axis offset trim low byte */

/* Self-Test Registers */
#define MPU6050_SELF_TEST_X             0x0DU  /* Self-test X (XG_TEST[4:0] + XA_TEST[4:2]) */
#define MPU6050_SELF_TEST_Y             0x0EU  /* Self-test Y (YG_TEST[4:0] + YA_TEST[4:2]) */
#define MPU6050_SELF_TEST_Z             0x0FU  /* Self-test Z (ZG_TEST[4:0] + ZA_TEST[4:2]) */
#define MPU6050_SELF_TEST_A             0x10U  /* Self-test accel low bits (XA/YA/ZA_TEST[1:0]) */

/* Gyro Offset Cancellation Registers */
#define MPU6050_XG_OFFS_USRH            0x13U  /* Gyro X-axis offset cancellation high byte */
#define MPU6050_XG_OFFS_USRL            0x14U  /* Gyro X-axis offset cancellation low byte */
#define MPU6050_YG_OFFS_USRH            0x15U  /* Gyro Y-axis offset cancellation high byte */
#define MPU6050_YG_OFFS_USRL            0x16U  /* Gyro Y-axis offset cancellation low byte */
#define MPU6050_ZG_OFFS_USRH            0x17U  /* Gyro Z-axis offset cancellation high byte */
#define MPU6050_ZG_OFFS_USRL            0x18U  /* Gyro Z-axis offset cancellation low byte */

/* Configuration Registers */
#define MPU6050_SMPLRT_DIV              0x19U  /* Sample rate divider (Sample Rate = Gyro_Output_Rate / (1 + SMPLRT_DIV)) */
#define MPU6050_CONFIG                  0x1AU  /* External sync (EXT_SYNC_SET) & DLPF_CFG */
#define MPU6050_GYRO_CONFIG             0x1BU  /* Gyroscope full scale range (FS_SEL) & self-test enable */
#define MPU6050_ACCEL_CONFIG            0x1CU  /* Accelerometer full scale range (AFS_SEL), self-test, HPF */
#define MPU6050_FF_THR                  0x1DU  /* Free-fall detection threshold */
#define MPU6050_FF_DUR                  0x1EU  /* Free-fall detection duration */
#define MPU6050_MOT_THR                 0x1FU  /* Motion detection threshold */
#define MPU6050_MOT_DUR                 0x20U  /* Motion detection duration */
#define MPU6050_ZRMOT_THR               0x21U  /* Zero-motion detection threshold */
#define MPU6050_ZRMOT_DUR               0x22U  /* Zero-motion detection duration */

/* FIFO and I2C Master Control */
#define MPU6050_FIFO_EN                 0x23U  /* FIFO buffer enable for sensor data streams */
#define MPU6050_I2C_MST_CTRL            0x24U  /* I2C Master multi-master, clock speed, and wait config */
#define MPU6050_I2C_SLV0_ADDR           0x25U  /* I2C Slave 0 physical address & read/write mode */
#define MPU6050_I2C_SLV0_REG            0x26U  /* I2C Slave 0 target register address */
#define MPU6050_I2C_SLV0_CTRL           0x27U  /* I2C Slave 0 enable, byte swap, and transfer length */
#define MPU6050_I2C_SLV1_ADDR           0x28U  /* I2C Slave 1 physical address & read/write mode */
#define MPU6050_I2C_SLV1_REG            0x29U  /* I2C Slave 1 target register address */
#define MPU6050_I2C_SLV1_CTRL           0x2AU  /* I2C Slave 1 enable, byte swap, and transfer length */
#define MPU6050_I2C_SLV2_ADDR           0x2BU  /* I2C Slave 2 physical address & read/write mode */
#define MPU6050_I2C_SLV2_REG            0x2CU  /* I2C Slave 2 target register address */
#define MPU6050_I2C_SLV2_CTRL           0x2DU  /* I2C Slave 2 enable, byte swap, and transfer length */
#define MPU6050_I2C_SLV3_ADDR           0x2EU  /* I2C Slave 3 physical address & read/write mode */
#define MPU6050_I2C_SLV3_REG            0x2FU  /* I2C Slave 3 target register address */
#define MPU6050_I2C_SLV3_CTRL           0x30U  /* I2C Slave 3 enable, byte swap, and transfer length */
#define MPU6050_I2C_SLV4_ADDR           0x31U  /* I2C Slave 4 physical address & read/write mode */
#define MPU6050_I2C_SLV4_REG            0x32U  /* I2C Slave 4 target register address */
#define MPU6050_I2C_SLV4_DO             0x33U  /* I2C Slave 4 data output byte for write operations */
#define MPU6050_I2C_SLV4_CTRL           0x34U  /* I2C Slave 4 enable, interrupt, and transfer controls */
#define MPU6050_I2C_SLV4_DI             0x35U  /* I2C Slave 4 data input byte read from external device */
#define MPU6050_I2C_MST_STATUS          0x36U  /* I2C Master status register & slave NACK/completion flags */

/* Interrupt Configuration & Status */
#define MPU6050_INT_PIN_CFG             0x37U  /* INT pin logic level, push-pull/open-drain, and bypass enable */
#define MPU6050_INT_ENABLE              0x38U  /* Interrupt source enable (Data Ready, Motion, FIFO overflow, ...) */
#define MPU6050_INT_STATUS              0x3AU  /* Interrupt status flags (cleared on read) */

/* Sensor Data Output Registers */
#define MPU6050_ACCEL_XOUT_H            0x3BU  /* Accelerometer X-axis measurement high byte */
#define MPU6050_ACCEL_XOUT_L            0x3CU  /* Accelerometer X-axis measurement low byte */
#define MPU6050_ACCEL_YOUT_H            0x3DU  /* Accelerometer Y-axis measurement high byte */
#define MPU6050_ACCEL_YOUT_L            0x3EU  /* Accelerometer Y-axis measurement low byte */
#define MPU6050_ACCEL_ZOUT_H            0x3FU  /* Accelerometer Z-axis measurement high byte */
#define MPU6050_ACCEL_ZOUT_L            0x40U  /* Accelerometer Z-axis measurement low byte */
#define MPU6050_TEMP_OUT_H              0x41U  /* Temperature measurement high byte */
#define MPU6050_TEMP_OUT_L              0x42U  /* Temperature measurement low byte */
#define MPU6050_GYRO_XOUT_H             0x43U  /* Gyroscope X-axis measurement high byte */
#define MPU6050_GYRO_XOUT_L             0x44U  /* Gyroscope X-axis measurement low byte */
#define MPU6050_GYRO_YOUT_H             0x45U  /* Gyroscope Y-axis measurement high byte */
#define MPU6050_GYRO_YOUT_L             0x46U  /* Gyroscope Y-axis measurement low byte */
#define MPU6050_GYRO_ZOUT_H             0x47U  /* Gyroscope Z-axis measurement high byte */
#define MPU6050_GYRO_ZOUT_L             0x48U  /* Gyroscope Z-axis measurement low byte */

/* External Sensor Data Registers (read via auxiliary I2C master) */
#define MPU6050_EXT_SENS_DATA_00        0x49U  /* External sensor data byte 00 */
#define MPU6050_EXT_SENS_DATA_01        0x4AU  /* External sensor data byte 01 */
#define MPU6050_EXT_SENS_DATA_02        0x4BU  /* External sensor data byte 02 */
#define MPU6050_EXT_SENS_DATA_03        0x4CU  /* External sensor data byte 03 */
#define MPU6050_EXT_SENS_DATA_04        0x4DU  /* External sensor data byte 04 */
#define MPU6050_EXT_SENS_DATA_05        0x4EU  /* External sensor data byte 05 */
#define MPU6050_EXT_SENS_DATA_06        0x4FU  /* External sensor data byte 06 */
#define MPU6050_EXT_SENS_DATA_07        0x50U  /* External sensor data byte 07 */
#define MPU6050_EXT_SENS_DATA_08        0x51U  /* External sensor data byte 08 */
#define MPU6050_EXT_SENS_DATA_09        0x52U  /* External sensor data byte 09 */
#define MPU6050_EXT_SENS_DATA_10        0x53U  /* External sensor data byte 10 */
#define MPU6050_EXT_SENS_DATA_11        0x54U  /* External sensor data byte 11 */
#define MPU6050_EXT_SENS_DATA_12        0x55U  /* External sensor data byte 12 */
#define MPU6050_EXT_SENS_DATA_13        0x56U  /* External sensor data byte 13 */
#define MPU6050_EXT_SENS_DATA_14        0x57U  /* External sensor data byte 14 */
#define MPU6050_EXT_SENS_DATA_15        0x58U  /* External sensor data byte 15 */
#define MPU6050_EXT_SENS_DATA_16        0x59U  /* External sensor data byte 16 */
#define MPU6050_EXT_SENS_DATA_17        0x5AU  /* External sensor data byte 17 */
#define MPU6050_EXT_SENS_DATA_18        0x5BU  /* External sensor data byte 18 */
#define MPU6050_EXT_SENS_DATA_19        0x5CU  /* External sensor data byte 19 */
#define MPU6050_EXT_SENS_DATA_20        0x5DU  /* External sensor data byte 20 */
#define MPU6050_EXT_SENS_DATA_21        0x5EU  /* External sensor data byte 21 */
#define MPU6050_EXT_SENS_DATA_22        0x5FU  /* External sensor data byte 22 */
#define MPU6050_EXT_SENS_DATA_23        0x60U  /* External sensor data byte 23 */

/* Motion Detect Status */
#define MPU6050_MOT_DETECT_STATUS       0x61U  /* Motion detection status (axis / direction flags) */

/* Slave Data Out Registers */
#define MPU6050_I2C_SLV0_DO             0x63U  /* Data out byte written to I2C Slave 0 */
#define MPU6050_I2C_SLV1_DO             0x64U  /* Data out byte written to I2C Slave 1 */
#define MPU6050_I2C_SLV2_DO             0x65U  /* Data out byte written to I2C Slave 2 */
#define MPU6050_I2C_SLV3_DO             0x66U  /* Data out byte written to I2C Slave 3 */

/* I2C Master Delay & Signal Path Reset */
#define MPU6050_I2C_MST_DELAY_CTRL      0x67U  /* Delay shadow register updates for external sensors */
#define MPU6050_SIGNAL_PATH_RESET       0x68U  /* Reset analog & digital signal paths of gyro, accel, temp */
#define MPU6050_MOT_DETECT_CTRL         0x69U  /* Motion detection control (accel power-on delay, FF/MOT decrement) */
#define MPU6050_USER_CTRL               0x6AU  /* Enable/disable FIFO, I2C Master, and reset FIFO/I2C/SIG_COND */

/* Power Management */
#define MPU6050_PWR_MGMT_1              0x6BU  /* Device reset, sleep mode, cycle mode, temp disable, clock source */
#define MPU6050_PWR_MGMT_2              0x6CU  /* Wake-up frequency & standby mode for individual gyro/accel axes */

/* FIFO Count & Read/Write */
#define MPU6050_FIFO_COUNTH             0x72U  /* Current FIFO buffer byte count high byte */
#define MPU6050_FIFO_COUNTL             0x73U  /* Current FIFO buffer byte count low byte */
#define MPU6050_FIFO_R_W                0x74U  /* FIFO read/write data buffer register */

/* Device Identification */
#define MPU6050_WHO_AM_I                0x75U  /* Device ID register (bits[6:1] of the I2C address, returns 0x68) */

/* Aliases for backwards compatibility with MPU6500 driver style */
#define MPU6050_WHO_AM_I_REG            MPU6050_WHO_AM_I
#define MPU6050_PWR_MGMT_1_REG          MPU6050_PWR_MGMT_1
#define MPU6050_SMPLRT_DIV_REG          MPU6050_SMPLRT_DIV
#define MPU6050_CONFIG_REG              MPU6050_CONFIG
#define MPU6050_GYRO_CONFIG_REG         MPU6050_GYRO_CONFIG
#define MPU6050_ACCEL_CONFIG_REG        MPU6050_ACCEL_CONFIG
#define MPU6050_ACCEL_XOUT_H_REG        MPU6050_ACCEL_XOUT_H

/* ============================================================================================================
**                                          REGISTER BIT FIELDS
** ============================================================================================================ */
/* CONFIG (0x1A) */
#define MPU6050_DLPF_CFG_260HZ          0x00U  /* Accel 260Hz / Gyro 256Hz, gyro rate 8kHz */
#define MPU6050_DLPF_CFG_184HZ          0x01U  /* Accel 184Hz / Gyro 188Hz, gyro rate 1kHz */
#define MPU6050_DLPF_CFG_94HZ           0x02U  /* Accel 94Hz  / Gyro 98Hz,  gyro rate 1kHz */
#define MPU6050_DLPF_CFG_44HZ           0x03U  /* Accel 44Hz  / Gyro 42Hz,  gyro rate 1kHz */
#define MPU6050_DLPF_CFG_21HZ           0x04U  /* Accel 21Hz  / Gyro 20Hz,  gyro rate 1kHz */
#define MPU6050_DLPF_CFG_10HZ           0x05U  /* Accel 10Hz  / Gyro 10Hz,  gyro rate 1kHz */
#define MPU6050_DLPF_CFG_5HZ            0x06U  /* Accel 5Hz   / Gyro 5Hz,   gyro rate 1kHz */

/* GYRO_CONFIG (0x1B) */
#define MPU6050_GYRO_FS_250DPS          (0U << 3)
#define MPU6050_GYRO_FS_500DPS          (1U << 3)
#define MPU6050_GYRO_FS_1000DPS         (2U << 3)
#define MPU6050_GYRO_FS_2000DPS         (3U << 3)
#define MPU6050_GYRO_ST_XYZ             0xE0U  /* XG_ST | YG_ST | ZG_ST self-test enable */

/* ACCEL_CONFIG (0x1C) */
#define MPU6050_ACCEL_FS_2G             (0U << 3)
#define MPU6050_ACCEL_FS_4G             (1U << 3)
#define MPU6050_ACCEL_FS_8G             (2U << 3)
#define MPU6050_ACCEL_FS_16G            (3U << 3)
#define MPU6050_ACCEL_ST_XYZ            0xE0U  /* XA_ST | YA_ST | ZA_ST self-test enable */

/* INT_PIN_CFG (0x37) */
#define MPU6050_INT_LEVEL_LOW           (1U << 7)  /* INT pin active low */
#define MPU6050_INT_OPEN_DRAIN          (1U << 6)  /* INT pin open-drain */
#define MPU6050_LATCH_INT_EN            (1U << 5)  /* INT pin level held until cleared */
#define MPU6050_INT_RD_CLEAR            (1U << 4)  /* INT status cleared on any read */
#define MPU6050_FSYNC_INT_LEVEL         (1U << 3)
#define MPU6050_FSYNC_INT_EN            (1U << 2)
#define MPU6050_I2C_BYPASS_EN           (1U << 1)  /* Aux I2C bus connected to host bus */

/* INT_ENABLE (0x38) / INT_STATUS (0x3A) */
#define MPU6050_INT_MOT_EN              (1U << 6)
#define MPU6050_INT_FIFO_OFLOW_EN       (1U << 4)
#define MPU6050_INT_I2C_MST_EN          (1U << 3)
#define MPU6050_INT_DATA_RDY_EN         (1U << 0)

/* USER_CTRL (0x6A) */
#define MPU6050_USER_FIFO_EN            (1U << 6)
#define MPU6050_USER_I2C_MST_EN         (1U << 5)
#define MPU6050_USER_I2C_IF_DIS         (1U << 4)
#define MPU6050_USER_FIFO_RESET         (1U << 2)
#define MPU6050_USER_I2C_MST_RESET      (1U << 1)
#define MPU6050_USER_SIG_COND_RESET     (1U << 0)

/* PWR_MGMT_1 (0x6B) */
#define MPU6050_PWR1_DEVICE_RESET       (1U << 7)
#define MPU6050_PWR1_SLEEP              (1U << 6)
#define MPU6050_PWR1_CYCLE              (1U << 5)
#define MPU6050_PWR1_TEMP_DIS           (1U << 3)
#define MPU6050_PWR1_CLKSEL_INT8M       0x00U  /* Internal 8MHz oscillator */
#define MPU6050_PWR1_CLKSEL_PLL_XGYRO   0x01U  /* PLL with X-axis gyro reference (recommended) */
#define MPU6050_PWR1_CLKSEL_PLL_YGYRO   0x02U
#define MPU6050_PWR1_CLKSEL_PLL_ZGYRO   0x03U

/* ============================================================================================================
**                                          SCALE FACTORS / HELPERS
** ============================================================================================================ */
#define MPU6050_ACCEL_SENS_2G           16384.0f   /* LSB/g */
#define MPU6050_ACCEL_SENS_4G           8192.0f
#define MPU6050_ACCEL_SENS_8G           4096.0f
#define MPU6050_ACCEL_SENS_16G          2048.0f

#define MPU6050_GYRO_SENS_250DPS        131.0f     /* LSB/(deg/s) */
#define MPU6050_GYRO_SENS_500DPS        65.5f
#define MPU6050_GYRO_SENS_1000DPS       32.8f
#define MPU6050_GYRO_SENS_2000DPS       16.4f

#define MPU6050_TEMP_TO_C(raw)          ((float)(raw) / 340.0f + 36.53f)

/* ============================================================================================================
**                                          DRIVER STRUCTURE
** ============================================================================================================ */
/* Raw sample (same layout as the 14-byte burst starting at ACCEL_XOUT_H) */
typedef struct
{
    int16_t accel[3];       /* X, Y, Z */
    int16_t temp;
    int16_t gyro[3];        /* X, Y, Z */
} MPU6050_raw_t;

typedef struct MPU6050_DRIVER_S MPU6050_driver_t;

struct MPU6050_DRIVER_S
{
    void *handle;

    /* Blocking API (polling, uses HAL_I2C_Mem_Read/Write = repeated start) */
    code_status_t (*init)(MPU6050_driver_t *self);
    code_status_t (*writeReg)(MPU6050_driver_t *self, uint8_t reg, uint8_t data);
    code_status_t (*readReg)(MPU6050_driver_t *self, uint8_t reg, uint8_t *data);
    code_status_t (*readRegs)(MPU6050_driver_t *self, uint8_t reg, uint8_t *data, uint16_t len);
    code_status_t (*who_am_i)(MPU6050_driver_t *self, uint8_t *id);
    code_status_t (*readRawData)(MPU6050_driver_t *self, int16_t *accel, int16_t *gyro);
    code_status_t (*readRaw)(MPU6050_driver_t *self, MPU6050_raw_t *raw);    /* accel + temp + gyro */
    code_status_t (*recover)(MPU6050_driver_t *self);                        /* re-init I2C after an error */
};

/***************************************************************************************************************
**                                         FUNCTION PROTOTYPES
***************************************************************************************************************/
MPU6050_driver_t *MPU6050_driver_create(void);

#ifdef __cplusplus
}
#endif

#endif /* _MPU6050_DRIVER_H_ */
