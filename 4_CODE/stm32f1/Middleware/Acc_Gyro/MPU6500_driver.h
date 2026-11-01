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




/* MPU6500 structure */
typedef struct MPU6500_DRIVER_S MPU6500_driver_t;

struct MPU6500_DRIVER_S
{
    void *handle;
    uint32_t (*init)(MPU6500_driver_t *self);
    uint32_t (*writeReg)(MPU6500_driver_t *self, uint8_t reg, uint8_t data);
    uint32_t (*readReg)(MPU6500_driver_t *self, uint8_t reg, uint8_t *data);
    uint32_t (*readRegs)(MPU6500_driver_t *self, uint8_t reg, uint8_t *data, uint16_t len);
    uint32_t (*who_am_i)(MPU6500_driver_t *self, uint8_t *id);
    uint32_t (*readRawData)(MPU6500_driver_t *self, int16_t *accel, int16_t *gyro);
};

/***************************************************************************************************************
**                                         FUNCTION PROTOTYPES
***************************************************************************************************************/
MPU6500_driver_t *MPU6500_driver_create(void);

#ifdef __cplusplus
}
#endif

#endif /* _MPU6500_DRIVER_H_ */