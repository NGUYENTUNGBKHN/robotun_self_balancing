/*
****************************************************************************************************************
 * @file       MPU6500_driver.h
 * @brief      MPU6500 I2C driver header for STM32F1
 * @date       2026/10/29
 * @author     [Gentantun] (nguyenthanhtung8196@gmail.com)
 * @details    I2C1: PB6=SCL, PB7=SDA
 * @copyright  Copyright (c) 2026 RoboTun
****************************************************************************************************************
*/
#ifndef _MPU6500_DRIVER_H_
#define _MPU6500_DRIVER_H_

#ifdef __cplusplus
extern "C"
{
#endif

/***************************************************************************************************************
**                                                   INCLUDES
***************************************************************************************************************/
#include <stdint.h>

/***************************************************************************************************************
**                                            TYPE DEFINITIONS
***************************************************************************************************************/

#define MPU6500_INSTANCE                I2C1                /* use I2C1 */
#define MPU6500_CLOCK_SPEED             400000              /* 400kHz */
#define MPU6500_DUTY_CYCLE              I2C_DUTYCYCLE_2     /* use duty cycle 2 */


#define MPU6500_I2C_ADDR                (0x68U << 1)        /* 7-bit address 0x68 shifted for HAL (AD0=0) */
#define MPU6500_I2C_ADDR_AD0_HIGH       (0x69U << 1)        /* 7-bit address 0x69 shifted for HAL (AD0=1) */
#define MPU6500_WHO_AM_I_VAL            0x70U               /* Default WHO_AM_I value for MPU6500 */
#define MPU6500_I2C_TIMEOUT_MS          100U

/* ============================================================================================================
**                                          MPU-6500 REGISTER MAP
** ============================================================================================================ */
/* Self-Test Registers */
#define MPU6500_SELF_TEST_X_GYRO        0x00U  /* Gyroscope X-axis self-test generated output */
#define MPU6500_SELF_TEST_Y_GYRO        0x01U  /* Gyroscope Y-axis self-test generated output */
#define MPU6500_SELF_TEST_Z_GYRO        0x02U  /* Gyroscope Z-axis self-test generated output */
#define MPU6500_SELF_TEST_X_ACCEL       0x0DU  /* Accelerometer X-axis self-test generated output */
#define MPU6500_SELF_TEST_Y_ACCEL       0x0EU  /* Accelerometer Y-axis self-test generated output */
#define MPU6500_SELF_TEST_Z_ACCEL       0x0FU  /* Accelerometer Z-axis self-test generated output */

/* Gyro Offset Cancellation Registers */
#define MPU6500_XG_OFFSET_H             0x13U  /* Gyro X-axis offset cancellation high byte */
#define MPU6500_XG_OFFSET_L             0x14U  /* Gyro X-axis offset cancellation low byte */
#define MPU6500_YG_OFFSET_H             0x15U  /* Gyro Y-axis offset cancellation high byte */
#define MPU6500_YG_OFFSET_L             0x16U  /* Gyro Y-axis offset cancellation low byte */
#define MPU6500_ZG_OFFSET_H             0x17U  /* Gyro Z-axis offset cancellation high byte */
#define MPU6500_ZG_OFFSET_L             0x18U  /* Gyro Z-axis offset cancellation low byte */

/* Configuration Registers */
#define MPU6500_SMPLRT_DIV              0x19U  /* Sample rate divider (Sample Rate = Internal_Sample_Rate / (1 + SMPLRT_DIV)) */
#define MPU6500_CONFIG                  0x1AU  /* External sync & DLPF (Digital Low Pass Filter) config for Gyro/Temp */
#define MPU6500_GYRO_CONFIG             0x1BU  /* Gyroscope full scale range (FS_SEL) & self-test enable */
#define MPU6500_ACCEL_CONFIG            0x1CU  /* Accelerometer full scale range (AFS_SEL) & self-test enable */
#define MPU6500_ACCEL_CONFIG_2          0x1DU  /* Accelerometer DLPF and rate configuration */
#define MPU6500_LP_ACCEL_ODR            0x1EU  /* Low power accelerometer output data rate control */
#define MPU6500_WOM_THR                 0x1FU  /* Wake-on-motion interrupt acceleration threshold */

/* FIFO and I2C Master Control */
#define MPU6500_FIFO_EN                 0x23U  /* FIFO buffer enable for sensor data streams */
#define MPU6500_I2C_MST_CTRL            0x24U  /* I2C Master multi-master, clock speed, and wait config */
#define MPU6500_I2C_SLV0_ADDR           0x25U  /* I2C Slave 0 physical address & read/write mode */
#define MPU6500_I2C_SLV0_REG            0x26U  /* I2C Slave 0 target register address */
#define MPU6500_I2C_SLV0_CTRL           0x27U  /* I2C Slave 0 enable, byte swap, and transfer length */
#define MPU6500_I2C_SLV1_ADDR           0x28U  /* I2C Slave 1 physical address & read/write mode */
#define MPU6500_I2C_SLV1_REG            0x29U  /* I2C Slave 1 target register address */
#define MPU6500_I2C_SLV1_CTRL           0x2AU  /* I2C Slave 1 enable, byte swap, and transfer length */
#define MPU6500_I2C_SLV2_ADDR           0x2BU  /* I2C Slave 2 physical address & read/write mode */
#define MPU6500_I2C_SLV2_REG            0x2CU  /* I2C Slave 2 target register address */
#define MPU6500_I2C_SLV2_CTRL           0x2DU  /* I2C Slave 2 enable, byte swap, and transfer length */
#define MPU6500_I2C_SLV3_ADDR           0x2EU  /* I2C Slave 3 physical address & read/write mode */
#define MPU6500_I2C_SLV3_REG            0x2FU  /* I2C Slave 3 target register address */
#define MPU6500_I2C_SLV3_CTRL           0x30U  /* I2C Slave 3 enable, byte swap, and transfer length */
#define MPU6500_I2C_SLV4_ADDR           0x31U  /* I2C Slave 4 physical address & read/write mode */
#define MPU6500_I2C_SLV4_REG            0x32U  /* I2C Slave 4 target register address */
#define MPU6500_I2C_SLV4_DO             0x33U  /* I2C Slave 4 data output byte for write operations */
#define MPU6500_I2C_SLV4_CTRL           0x34U  /* I2C Slave 4 enable, interrupt, and transfer controls */
#define MPU6500_I2C_SLV4_DI             0x35U  /* I2C Slave 4 data input byte read from external device */
#define MPU6500_I2C_MST_STATUS          0x36U  /* I2C Master status register & slave NACK/completion flags */

/* Interrupt Configuration & Status */
#define MPU6500_INT_PIN_CFG             0x37U  /* INT pin logic level, push-pull/open-drain, and bypass enable */
#define MPU6500_INT_ENABLE              0x38U  /* Interrupt source enable (Data Ready, WOM, FIFO overflow, etc.) */
#define MPU6500_INT_STATUS              0x3AU  /* Interrupt status flags (cleared on read) */

/* Sensor Data Output Registers */
#define MPU6500_ACCEL_XOUT_H            0x3BU  /* Accelerometer X-axis measurement high byte */
#define MPU6500_ACCEL_XOUT_L            0x3CU  /* Accelerometer X-axis measurement low byte */
#define MPU6500_ACCEL_YOUT_H            0x3DU  /* Accelerometer Y-axis measurement high byte */
#define MPU6500_ACCEL_YOUT_L            0x3EU  /* Accelerometer Y-axis measurement low byte */
#define MPU6500_ACCEL_ZOUT_H            0x3FU  /* Accelerometer Z-axis measurement high byte */
#define MPU6500_ACCEL_ZOUT_L            0x40U  /* Accelerometer Z-axis measurement low byte */
#define MPU6500_TEMP_OUT_H              0x41U  /* Temperature measurement high byte */
#define MPU6500_TEMP_OUT_L              0x42U  /* Temperature measurement low byte */
#define MPU6500_GYRO_XOUT_H             0x43U  /* Gyroscope X-axis measurement high byte */
#define MPU6500_GYRO_XOUT_L             0x44U  /* Gyroscope X-axis measurement low byte */
#define MPU6500_GYRO_YOUT_H             0x45U  /* Gyroscope Y-axis measurement high byte */
#define MPU6500_GYRO_YOUT_L             0x46U  /* Gyroscope Y-axis measurement low byte */
#define MPU6500_GYRO_ZOUT_H             0x47U  /* Gyroscope Z-axis measurement high byte */
#define MPU6500_GYRO_ZOUT_L             0x48U  /* Gyroscope Z-axis measurement low byte */

/* External Sensor Data Registers (read via auxiliary I2C master) */
#define MPU6500_EXT_SENS_DATA_00        0x49U  /* External sensor data byte 00 */
#define MPU6500_EXT_SENS_DATA_01        0x4AU  /* External sensor data byte 01 */
#define MPU6500_EXT_SENS_DATA_02        0x4BU  /* External sensor data byte 02 */
#define MPU6500_EXT_SENS_DATA_03        0x4CU  /* External sensor data byte 03 */
#define MPU6500_EXT_SENS_DATA_04        0x4DU  /* External sensor data byte 04 */
#define MPU6500_EXT_SENS_DATA_05        0x4EU  /* External sensor data byte 05 */
#define MPU6500_EXT_SENS_DATA_06        0x4FU  /* External sensor data byte 06 */
#define MPU6500_EXT_SENS_DATA_07        0x50U  /* External sensor data byte 07 */
#define MPU6500_EXT_SENS_DATA_08        0x51U  /* External sensor data byte 08 */
#define MPU6500_EXT_SENS_DATA_09        0x52U  /* External sensor data byte 09 */
#define MPU6500_EXT_SENS_DATA_10        0x53U  /* External sensor data byte 10 */
#define MPU6500_EXT_SENS_DATA_11        0x54U  /* External sensor data byte 11 */
#define MPU6500_EXT_SENS_DATA_12        0x55U  /* External sensor data byte 12 */
#define MPU6500_EXT_SENS_DATA_13        0x56U  /* External sensor data byte 13 */
#define MPU6500_EXT_SENS_DATA_14        0x57U  /* External sensor data byte 14 */
#define MPU6500_EXT_SENS_DATA_15        0x58U  /* External sensor data byte 15 */
#define MPU6500_EXT_SENS_DATA_16        0x59U  /* External sensor data byte 16 */
#define MPU6500_EXT_SENS_DATA_17        0x5AU  /* External sensor data byte 17 */
#define MPU6500_EXT_SENS_DATA_18        0x5BU  /* External sensor data byte 18 */
#define MPU6500_EXT_SENS_DATA_19        0x5CU  /* External sensor data byte 19 */
#define MPU6500_EXT_SENS_DATA_20        0x5DU  /* External sensor data byte 20 */
#define MPU6500_EXT_SENS_DATA_21        0x5EU  /* External sensor data byte 21 */
#define MPU6500_EXT_SENS_DATA_22        0x5FU  /* External sensor data byte 22 */
#define MPU6500_EXT_SENS_DATA_23        0x60U  /* External sensor data byte 23 */

/* Slave Data Out Registers */
#define MPU6500_I2C_SLV0_DO             0x63U  /* Data out byte written to I2C Slave 0 */
#define MPU6500_I2C_SLV1_DO             0x64U  /* Data out byte written to I2C Slave 1 */
#define MPU6500_I2C_SLV2_DO             0x65U  /* Data out byte written to I2C Slave 2 */
#define MPU6500_I2C_SLV3_DO             0x66U  /* Data out byte written to I2C Slave 3 */

/* I2C Master Delay & Signal Path Reset */
#define MPU6500_I2C_MST_DELAY_CTRL      0x67U  /* Delay shadow register updates for external sensors */
#define MPU6500_SIGNAL_PATH_RESET       0x68U  /* Reset analog & digital signal paths of gyro, accel, temp */
#define MPU6500_ACCEL_INTEL_CTRL        0x69U  /* Accelerometer intelligence / wake-on-motion control */
#define MPU6500_USER_CTRL               0x6AU  /* Enable/disable FIFO, I2C Master, and reset DMP/FIFO/I2C */

/* Power Management */
#define MPU6500_PWR_MGMT_1              0x6BU  /* Device reset, sleep mode, cycle mode, and clock source */
#define MPU6500_PWR_MGMT_2              0x6CU  /* Standby mode control for individual gyro and accel axes */

/* FIFO Count & Read/Write */
#define MPU6500_FIFO_COUNTH             0x72U  /* Current FIFO buffer byte count high byte */
#define MPU6500_FIFO_COUNTL             0x73U  /* Current FIFO buffer byte count low byte */
#define MPU6500_FIFO_R_W                0x74U  /* FIFO read/write data buffer register */

/* Device Identification */
#define MPU6500_WHO_AM_I                0x75U  /* Device ID register (returns 0x70 for MPU-6500) */

/* Accelerometer Offset Cancellation Registers */
#define MPU6500_XA_OFFSET_H             0x77U  /* Accelerometer X-axis offset cancellation high byte */
#define MPU6500_XA_OFFSET_L             0x78U  /* Accelerometer X-axis offset cancellation low byte */
#define MPU6500_YA_OFFSET_H             0x7AU  /* Accelerometer Y-axis offset cancellation high byte */
#define MPU6500_YA_OFFSET_L             0x7BU  /* Accelerometer Y-axis offset cancellation low byte */
#define MPU6500_ZA_OFFSET_H             0x7DU  /* Accelerometer Z-axis offset cancellation high byte */
#define MPU6500_ZA_OFFSET_L             0x7EU  /* Accelerometer Z-axis offset cancellation low byte */

/* Aliases for backwards compatibility */
#define MPU6500_WHO_AM_I_REG            MPU6500_WHO_AM_I
#define MPU6500_PWR_MGMT_1_REG          MPU6500_PWR_MGMT_1
#define MPU6500_SMPLRT_DIV_REG          MPU6500_SMPLRT_DIV
#define MPU6500_CONFIG_REG              MPU6500_CONFIG
#define MPU6500_GYRO_CONFIG_REG         MPU6500_GYRO_CONFIG
#define MPU6500_ACCEL_CONFIG_REG        MPU6500_ACCEL_CONFIG
#define MPU6500_ACCEL_XOUT_H_REG        MPU6500_ACCEL_XOUT_H


/* MPU6500 structure */
typedef struct MPU6500_DRIVER_S MPU6500_driver_t;

struct MPU6500_DRIVER_S
{
    void *handle;
    code_status_t (*init)(MPU6500_driver_t *self);
    code_status_t (*writeReg)(MPU6500_driver_t *self, uint8_t reg, uint8_t data);
    code_status_t (*readReg)(MPU6500_driver_t *self, uint8_t reg, uint8_t *data);
    code_status_t (*readRegs)(MPU6500_driver_t *self, uint8_t reg, uint8_t *data, uint16_t len);
    code_status_t (*who_am_i)(MPU6500_driver_t *self, uint8_t *id);
    code_status_t (*readRawData)(MPU6500_driver_t *self, int16_t *accel, int16_t *gyro);
};

/***************************************************************************************************************
**                                         FUNCTION PROTOTYPES
***************************************************************************************************************/
MPU6500_driver_t *MPU6500_driver_create(void);

#ifdef __cplusplus
}
#endif

#endif /* _MPU6500_DRIVER_H_ */