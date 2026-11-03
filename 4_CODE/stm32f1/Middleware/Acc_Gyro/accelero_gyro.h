/*
****************************************************************************************************************
 * @file       accelero_gyro.h
 * @brief      Complementary and Kalman Filter interface for MPU6500
 * @date       2026/10/29
 * @author     [Gentantun] (nguyenthanhtung8196@gmail.com)
 * @details    Calculates Roll/Pitch angles using Complementary & Kalman filters
 * @copyright  Copyright (c) 2026 RoboTun
****************************************************************************************************************
*/
#ifndef _ACCELERO_GYRO_H_
#define _ACCELERO_GYRO_H_

#ifdef __cplusplus
extern "C"
{
#endif

/***************************************************************************************************************
**                                                   INCLUDES
***************************************************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "MPU6500_driver.h"

/***************************************************************************************************************
**                                            MACROS & CONSTANTS
***************************************************************************************************************/
/* Scale factors based on MPU6500 configuration:
 * ACCEL: +/- 2g  -> 16384 LSB/g
 * GYRO : +/- 500 deg/s -> 65.5 LSB/(deg/s)
 */
#define ACCEL_SCALE_FACTOR_2G       (16384.0f)
#define GYRO_SCALE_FACTOR_500DPS    (65.5f)

#define RAD_TO_DEG                  (57.29577951308232f)

/***************************************************************************************************************
**                                            TYPE DEFINITIONS
***************************************************************************************************************/

/**
 * @brief 1D Kalman Filter state structure
 */
typedef struct
{
    float Q_angle;      /* Process noise variance for the accelerometer angle */
    float Q_bias;       /* Process noise variance for the gyro bias */
    float R_measure;    /* Measurement noise variance */
    
    float angle;        /* Calculated angle (state x_0) */
    float bias;         /* Calculated gyro bias (state x_1) */
    float rate;         /* Unbiased angular velocity */

    float P[2][2];      /* Error covariance matrix */
} kalman_filter_t;

/**
 * @brief Complementary Filter structure
 */
typedef struct
{
    float alpha;        /* Weighting factor: angle = alpha*(angle + gyro*dt) + (1-alpha)*accel_angle */
    float angle;        /* Filtered angle output */
} complementary_filter_t;

/**
 * @brief IMU attitude estimation container
 */
typedef struct
{
    /* Calibration offsets */
    int32_t accel_offset[3];
    int32_t gyro_offset[3];
    bool    is_calibrated;

    /* Scaled accelerometer (g) & gyro (deg/s) */
    float ax, ay, az;
    float gx, gy, gz;

    /* Accelerometer calculated angles (deg) */
    float accel_roll;
    float accel_pitch;

    /* Filter outputs for Roll */
    float comp_roll;
    float kalman_roll;

    /* Filter outputs for Pitch */
    float comp_pitch;
    float kalman_pitch;

    /* Internal filter states */
    complementary_filter_t cf_roll;
    complementary_filter_t cf_pitch;
    kalman_filter_t        kf_roll;
    kalman_filter_t        kf_pitch;

    /* Driver handle */
    MPU6500_driver_t *driver;
} imu_sensor_t;

/***************************************************************************************************************
**                                         FUNCTION PROTOTYPES
***************************************************************************************************************/

/* Kalman Filter core routines */
void Kalman_Init(kalman_filter_t *kf, float Q_angle, float Q_bias, float R_measure);
float Kalman_Update(kalman_filter_t *kf, float new_angle, float new_rate, float dt);

/* Complementary Filter core routines */
void Complementary_Init(complementary_filter_t *cf, float alpha);
float Complementary_Update(complementary_filter_t *cf, float new_angle, float new_rate, float dt);

/* IMU High-level Management functions */
void IMU_Init(imu_sensor_t *imu, MPU6500_driver_t *driver);
void IMU_Calibrate(imu_sensor_t *imu, uint16_t sample_count);
void IMU_Update(imu_sensor_t *imu, float dt);

#ifdef __cplusplus
}
#endif

#endif /* _ACCELERO_GYRO_H_ */
