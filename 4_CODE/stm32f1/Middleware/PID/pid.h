/*
****************************************************************************************************************
 * @file       pid.h
 * @brief      Discrete PID controller implementation for self-balancing robot
 * @date       2026/11/05
 * @author     [Gentantun] (nguyenthanhtung8196@gmail.com)
 * @details    Provides 3-stage cascaded control architecture:
 *               1. Position Loop:  Position error  -> Target velocity (RPM)
 *               2. Velocity Loop:  Velocity error  -> Target tilt offset (deg)
 *               3. Balance Loop:   Tilt + Gyro Y   -> Motor PWM duty (-1000..+1000)
 * @copyright  Copyright (c) 2026 RoboTun
****************************************************************************************************************
*/
#ifndef _PID_H_
#define _PID_H_

#ifdef __cplusplus
extern "C"
{
#endif

/***************************************************************************************************************
**                                                   INCLUDES
***************************************************************************************************************/
#include <stdint.h>
#include <stdbool.h>

/***************************************************************************************************************
**                                            TYPE DEFINITIONS
***************************************************************************************************************/

/**
 * @brief PID controller state and parameter structure
 */
typedef struct
{
    /* Gains */
    float Kp;               /* Proportional gain */
    float Ki;               /* Integral gain */
    float Kd;               /* Derivative gain */

    /* Setpoint and inputs */
    float target;           /* Desired setpoint */
    float error;            /* Current error (target - measurement) */
    float prev_error;       /* Previous step error */

    /* Integrator state */
    float integral;         /* Accumulated integral term */
    float integral_limit;   /* Maximum allowable integral term (anti-windup) */

    /* Output limits */
    float out_min;          /* Minimum output limit */
    float out_max;          /* Maximum output limit */

    /* Output */
    float output;           /* Controller calculated output */
} pid_controller_t;

/***************************************************************************************************************
**                                         FUNCTION PROTOTYPES
***************************************************************************************************************/

/**
 * @brief  Initialize a generic PID controller
 * @param  pid: Pointer to pid_controller_t structure
 * @param  Kp: Proportional gain
 * @param  Ki: Integral gain
 * @param  Kd: Derivative gain
 * @param  integral_limit: Anti-windup clamping threshold
 * @param  out_min: Lower saturation limit
 * @param  out_max: Upper saturation limit
 */
void PID_Init(pid_controller_t *pid, float Kp, float Ki, float Kd, 
              float integral_limit, float out_min, float out_max);

/**
 * @brief  Reset integral term and previous error of PID controller
 * @param  pid: Pointer to pid_controller_t structure
 */
void PID_Reset(pid_controller_t *pid);

/**
 * @brief  Standard positional PID update
 * @param  pid: Pointer to pid_controller_t structure
 * @param  measurement: Current process variable
 * @param  dt: Time step in seconds
 * @return Computed PID output
 */
float PID_Update(pid_controller_t *pid, float measurement, float dt);

/**
 * @brief  Balance PID loop (PD on angle and direct gyro angular rate)
 * @details For balancing robots, using the direct gyro angular velocity
 *          instead of (error - prev_error)/dt gives a much cleaner derivative
 *          without numerical differentiation noise.
 * @param  pid: Pointer to balance pid_controller_t
 * @param  angle: Current filtered pitch angle in degrees
 * @param  target_angle: Target balancing angle in degrees (mechanical zero balance point)
 * @param  gyro_rate: Angular velocity from gyro (e.g. deg/s)
 * @param  dt: Sampling period in seconds
 * @return Computed motor command (-1000 to +1000)
 */
float PID_BalanceUpdate(pid_controller_t *pid, float angle, float target_angle, 
                        float gyro_rate, float dt);

/**
 * @brief  Velocity PI loop for wheel encoder speed feedback
 * @param  pid: Pointer to velocity pid_controller_t
 * @param  current_speed: Measured wheel speed (in RPM)
 * @param  target_speed: Desired wheel speed (from position loop or manual command)
 * @param  dt: Sampling period in seconds
 * @return Tilt angle adjustment setpoint in degrees (added to target balance angle)
 */
float PID_VelocityUpdate(pid_controller_t *pid, float current_speed, 
                         float target_speed, float dt);

/**
 * @brief  Position PID loop for wheel distance/angle feedback (outer loop)
 * @param  pid: Pointer to position pid_controller_t
 * @param  current_position: Measured wheel position (e.g. degrees or ticks)
 * @param  target_position: Desired wheel position
 * @param  dt: Sampling period in seconds
 * @return Desired velocity in RPM (fed into PID_VelocityUpdate)
 */
float PID_PositionUpdate(pid_controller_t *pid, float current_position, 
                         float target_position, float dt);

#ifdef __cplusplus
}
#endif

#endif /* _PID_H_ */
