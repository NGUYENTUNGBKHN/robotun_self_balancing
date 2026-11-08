/*
****************************************************************************************************************
 * @file       accelero_gyro.c
 * @brief      Complementary and Kalman Filter implementation for MPU6500
 * @date       2026/10/29
 * @author     [Gentantun] (nguyenthanhtung8196@gmail.com)
 * @details    Provides attitude estimation (Roll & Pitch) using both Complementary and Kalman algorithms
 * @copyright  Copyright (c) 2026 RoboTun
****************************************************************************************************************
*/

/***************************************************************************************************************
**                                                   INCLUDES
***************************************************************************************************************/
#include <math.h>
#include "common.h"
#include <string.h>
#include "accelero_gyro.h"
#include "stm32f1xx_hal.h"
#include "tx_api.h"

/***************************************************************************************************************
**                                          INTERNAL MACRO DEFINITIONS
***************************************************************************************************************/
/* Default filter parameters:
 * Complementary filter: alpha = 0.98f (98% gyro integration, 2% accel correction)
 * Kalman filter:
 *   Q_angle   = 0.001f  (variance of angle noise)
 *   Q_bias    = 0.003f  (variance of gyro bias noise)
 *   R_measure = 0.03f   (measurement noise variance from accelerometer)
 */
#define COMPLEMENTARY_ALPHA_DEFAULT     (0.98f)
#define KALMAN_Q_ANGLE_DEFAULT          (0.001f)
#define KALMAN_Q_BIAS_DEFAULT           (0.003f)
#define KALMAN_R_MEASURE_DEFAULT        (0.03f)

/***************************************************************************************************************
**                                        KALMAN FILTER IMPLEMENTATION
***************************************************************************************************************/

/**
 * @brief  Initialize 1D Kalman Filter instance
 * @param  kf: Pointer to kalman_filter_t structure
 * @param  Q_angle: Process noise variance for the accelerometer angle
 * @param  Q_bias: Process noise variance for the gyro bias
 * @param  R_measure: Measurement noise variance
 */
void Kalman_Init(kalman_filter_t *kf, float Q_angle, float Q_bias, float R_measure)
{
    if (kf == NULL)
    {
        return;
    }

    kf->Q_angle   = Q_angle;
    kf->Q_bias    = Q_bias;
    kf->R_measure = R_measure;

    kf->angle = 0.0f;
    kf->bias  = 0.0f;
    kf->rate  = 0.0f;

    kf->P[0][0] = 0.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 0.0f;
}

/**
 * @brief  Kalman Filter update routine
 * @param  kf: Pointer to kalman_filter_t structure
 * @param  new_angle: Angle measured by accelerometer (in degrees)
 * @param  new_rate: Angular velocity measured by gyro (in deg/s)
 * @param  dt: Time delta since last update (in seconds)
 * @return Estimated angle in degrees
 */
float Kalman_Update(kalman_filter_t *kf, float new_angle, float new_rate, float dt)
{
    if (kf == NULL || dt <= 0.0f)
    {
        return (kf != NULL) ? kf->angle : 0.0f;
    }

    /* Step 1: Predict state
     * rate = new_rate - bias
     * angle = angle + rate * dt
     */
    kf->rate = new_rate - kf->bias;
    kf->angle += dt * kf->rate;

    /* Step 2: Update estimation error covariance matrix - Project ahead
     * P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle)
     * P[0][1] -= dt * P[1][1]
     * P[1][0] -= dt * P[1][1]
     * P[1][1] += Q_bias * dt
     */
    kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * dt;

    /* Step 3: Measurement update - Innovation covariance
     * S = P[0][0] + R_measure
     */
    float S = kf->P[0][0] + kf->R_measure;

    /* Step 4: Calculate Kalman Gain
     * K[0] = P[0][0] / S
     * K[1] = P[1][0] / S
     */
    float K[2];
    K[0] = kf->P[0][0] / S;
    K[1] = kf->P[1][0] / S;

    /* Step 5: Calculate innovation / measurement residual
     * y = new_angle - angle
     */
    float y = new_angle - kf->angle;

    /* Step 6: Update state estimate
     * angle += K[0] * y
     * bias  += K[1] * y
     */
    kf->angle += K[0] * y;
    kf->bias  += K[1] * y;

    /* Step 7: Update error covariance matrix
     * P[0][0] -= K[0] * P[0][0]
     * P[0][1] -= K[0] * P[0][1]
     * P[1][0] -= K[1] * P[0][0]
     * P[1][1] -= K[1] * P[0][1]
     */
    float P00_temp = kf->P[0][0];
    float P01_temp = kf->P[0][1];

    kf->P[0][0] -= K[0] * P00_temp;
    kf->P[0][1] -= K[0] * P01_temp;
    kf->P[1][0] -= K[1] * P00_temp;
    kf->P[1][1] -= K[1] * P01_temp;

    return kf->angle;
}

/***************************************************************************************************************
**                                    COMPLEMENTARY FILTER IMPLEMENTATION
***************************************************************************************************************/

/**
 * @brief  Initialize Complementary Filter
 * @param  cf: Pointer to complementary_filter_t structure
 * @param  alpha: Filter weighting coefficient (typically 0.95 - 0.99)
 */
void Complementary_Init(complementary_filter_t *cf, float alpha)
{
    if (cf == NULL)
    {
        return;
    }

    cf->alpha = alpha;
    cf->angle = 0.0f;
}

/**
 * @brief  Complementary Filter update routine
 * @param  cf: Pointer to complementary_filter_t structure
 * @param  new_angle: Accelerometer measured angle in degrees
 * @param  new_rate: Gyroscope measured rate in deg/s
 * @param  dt: Time step in seconds
 * @return Filtered angle in degrees
 */
float Complementary_Update(complementary_filter_t *cf, float new_angle, float new_rate, float dt)
{
    if (cf == NULL || dt <= 0.0f)
    {
        return (cf != NULL) ? cf->angle : 0.0f;
    }

    /* High-pass filter for gyro integration + low-pass filter for accelerometer */
    cf->angle = cf->alpha * (cf->angle + new_rate * dt) + (1.0f - cf->alpha) * new_angle;

    return cf->angle;
}

/***************************************************************************************************************
**                                        HIGH LEVEL IMU IMPLEMENTATION
***************************************************************************************************************/

/**
 * @brief  Initialize IMU container, filters, and hook to MPU6500 driver
 * @param  imu: Pointer to imu_sensor_t instance
 * @param  driver: Pointer to MPU6500 driver instance
 */
void IMU_Init(imu_sensor_t *imu, MPU6500_driver_t *driver)
{
    if (imu == NULL)
    {
        return;
    }

    memset(imu, 0, sizeof(imu_sensor_t));
    imu->driver = driver;

    /* Initialize filters with proven default parameters */
    Complementary_Init(&imu->cf_roll, COMPLEMENTARY_ALPHA_DEFAULT);
    Complementary_Init(&imu->cf_pitch, COMPLEMENTARY_ALPHA_DEFAULT);

    Kalman_Init(&imu->kf_roll, KALMAN_Q_ANGLE_DEFAULT, KALMAN_Q_BIAS_DEFAULT, KALMAN_R_MEASURE_DEFAULT);
    Kalman_Init(&imu->kf_pitch, KALMAN_Q_ANGLE_DEFAULT, KALMAN_Q_BIAS_DEFAULT, KALMAN_R_MEASURE_DEFAULT);
}

/**
 * @brief  Calibrate zero-rate offset for Gyroscope and Accelerometer
 * @note   Robot MUST BE stationary on a level surface during calibration!
 * @param  imu: Pointer to imu_sensor_t instance
 * @param  sample_count: Number of samples to average (recommended: 500 - 1000)
 */
void IMU_Calibrate(imu_sensor_t *imu, uint16_t sample_count)
{
    if (imu == NULL || imu->driver == NULL || sample_count == 0)
    {
        return;
    }

    int16_t raw_accel[3];
    int16_t raw_gyro[3];
    int64_t sum_ax = 0, sum_ay = 0, sum_az = 0;
    int64_t sum_gx = 0, sum_gy = 0, sum_gz = 0;

    for (uint16_t i = 0; i < sample_count; i++)
    {
        if (imu->driver->readRawData(imu->driver, raw_accel, raw_gyro) == ACE_SUCCESS)
        {
            sum_ax += raw_accel[0];
            sum_ay += raw_accel[1];
            sum_az += raw_accel[2];

            sum_gx += raw_gyro[0];
            sum_gy += raw_gyro[1];
            sum_gz += raw_gyro[2];
        }
        tx_thread_sleep(1U);
    }

    /* Gyro offsets (should be zero when stationary) */
    imu->gyro_offset[0] = (int32_t)(sum_gx / sample_count);
    imu->gyro_offset[1] = (int32_t)(sum_gy / sample_count);
    imu->gyro_offset[2] = (int32_t)(sum_gz / sample_count);

    /* Accel offsets: assuming Z-axis is pointing UP against gravity (+1g = 16384 LSB) */
    imu->accel_offset[0] = (int32_t)(sum_ax / sample_count);
    imu->accel_offset[1] = (int32_t)(sum_ay / sample_count);
    imu->accel_offset[2] = (int32_t)((sum_az / sample_count) - (int32_t)ACCEL_SCALE_FACTOR_2G);

    imu->is_calibrated = true;
}

/**
 * @brief  Read latest IMU data, apply calibration, and update both filters
 * @param  imu: Pointer to imu_sensor_t instance
 * @param  dt: Time interval in seconds (e.g., 0.005f for 200Hz loop)
 */
void IMU_Update(imu_sensor_t *imu, float dt)
{
    if (imu == NULL || imu->driver == NULL || dt <= 0.0f)
    {
        return;
    }

    int16_t raw_accel[3];
    int16_t raw_gyro[3];

    if (imu->driver->readRawData(imu->driver, raw_accel, raw_gyro) != ACE_SUCCESS)
    {
        return;
    }

    /* 1. Apply calibration offsets */
    int32_t cal_ax = raw_accel[0] - imu->accel_offset[0];
    int32_t cal_ay = raw_accel[1] - imu->accel_offset[1];
    int32_t cal_az = raw_accel[2] - imu->accel_offset[2];

    int32_t cal_gx = raw_gyro[0] - imu->gyro_offset[0];
    int32_t cal_gy = raw_gyro[1] - imu->gyro_offset[1];
    int32_t cal_gz = raw_gyro[2] - imu->gyro_offset[2];

    /* 2. Convert to physical units: g and deg/s */
    imu->ax = (float)cal_ax / ACCEL_SCALE_FACTOR_2G;
    imu->ay = (float)cal_ay / ACCEL_SCALE_FACTOR_2G;
    imu->az = (float)cal_az / ACCEL_SCALE_FACTOR_2G;

    imu->gx = (float)cal_gx / GYRO_SCALE_FACTOR_500DPS;
    imu->gy = (float)cal_gy / GYRO_SCALE_FACTOR_500DPS;
    imu->gz = (float)cal_gz / GYRO_SCALE_FACTOR_500DPS;

    /* 3. Calculate pitch and roll angles from Accelerometer (degrees)
     * Roll  = atan2(ay, az)
     * Pitch = atan2(-ax, sqrt(ay^2 + az^2))
     */
    imu->accel_roll  = atan2f(imu->ay, imu->az) * RAD_TO_DEG;
    imu->accel_pitch = atan2f(-imu->ax, sqrtf(imu->ay * imu->ay + imu->az * imu->az)) * RAD_TO_DEG;

    /* 4. Update Complementary Filter */
    imu->comp_roll  = Complementary_Update(&imu->cf_roll, imu->accel_roll, imu->gx, dt);
    imu->comp_pitch = Complementary_Update(&imu->cf_pitch, imu->accel_pitch, imu->gy, dt);

    /* 5. Update Kalman Filter */
    imu->kalman_roll  = Kalman_Update(&imu->kf_roll, imu->accel_roll, imu->gx, dt);
    imu->kalman_pitch = Kalman_Update(&imu->kf_pitch, imu->accel_pitch, imu->gy, dt);
}
