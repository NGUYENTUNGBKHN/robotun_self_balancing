/*
****************************************************************************************************************
 * @file       pid.c
 * @brief      Discrete PID controller implementation for self-balancing robot
 * @date       2026/11/04
 * @author     [Gentantun] (nguyenthanhtung8196@gmail.com)
 * @details    Implements standard PID, direct gyro rate balance PD, and velocity PI loop
 * @copyright  Copyright (c) 2026 RoboTun
****************************************************************************************************************
*/

/***************************************************************************************************************
**                                                   INCLUDES
***************************************************************************************************************/
#include "pid.h"
#include <string.h>

/***************************************************************************************************************
**                                             FUNCTION DEFINITIONS
***************************************************************************************************************/

/**
 * @brief  Initialize a generic PID controller
 */
void PID_Init(pid_controller_t *pid, float Kp, float Ki, float Kd, 
              float integral_limit, float out_min, float out_max)
{
    if (pid == NULL)
    {
        return;
    }

    memset(pid, 0, sizeof(pid_controller_t));

    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    pid->integral_limit = integral_limit;
    pid->out_min        = out_min;
    pid->out_max        = out_max;
}

/**
 * @brief  Reset integral term and previous error of PID controller
 */
void PID_Reset(pid_controller_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
    pid->output     = 0.0f;
}

/**
 * @brief  Standard positional PID update
 */
float PID_Update(pid_controller_t *pid, float measurement, float dt)
{
    if (pid == NULL || dt <= 0.0f)
    {
        return 0.0f;
    }

    /* 1. Calculate error */
    pid->error = pid->target - measurement;

    /* 2. Proportional term */
    float p_out = pid->Kp * pid->error;

    /* 3. Integral term with anti-windup clamping */
    pid->integral += pid->Ki * pid->error * dt;
    if (pid->integral > pid->integral_limit)
    {
        pid->integral = pid->integral_limit;
    }
    else if (pid->integral < -pid->integral_limit)
    {
        pid->integral = -pid->integral_limit;
    }
    float i_out = pid->integral;

    /* 4. Derivative term */
    float derivative = (pid->error - pid->prev_error) / dt;
    float d_out = pid->Kd * derivative;

    /* 5. Compute total output and saturate */
    pid->output = p_out + i_out + d_out;
    if (pid->output > pid->out_max)
    {
        pid->output = pid->out_max;
    }
    else if (pid->output < pid->out_min)
    {
        pid->output = pid->out_min;
    }

    /* Save previous error */
    pid->prev_error = pid->error;

    return pid->output;
}

/**
 * @brief  Balance PID loop (PD on angle and direct gyro angular rate)
 * @details Balance loop: Output = Kp * (angle - target) + Kd * gyro_rate
 */
float PID_BalanceUpdate(pid_controller_t *pid, float angle, float target_angle, 
                        float gyro_rate, float dt)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->target = target_angle;
    pid->error  = angle - target_angle;

    /* Proportional term (restoring torque proportional to inclination) */
    float p_out = pid->Kp * pid->error;

    /* Optional Integral term for steady-state tilt correction */
    if (pid->Ki != 0.0f && dt > 0.0f)
    {
        pid->integral += pid->Ki * pid->error * dt;
        if (pid->integral > pid->integral_limit)
        {
            pid->integral = pid->integral_limit;
        }
        else if (pid->integral < -pid->integral_limit)
        {
            pid->integral = -pid->integral_limit;
        }
    }
    else
    {
        pid->integral = 0.0f;
    }
    float i_out = pid->integral;

    /* Derivative term using clean gyro angular velocity directly */
    float d_out = pid->Kd * gyro_rate;

    /* Total control command to motors */
    pid->output = p_out + i_out + d_out;

    /* Clamp to limits (e.g. -1000 to +1000 PWM) */
    if (pid->output > pid->out_max)
    {
        pid->output = pid->out_max;
    }
    else if (pid->output < pid->out_min)
    {
        pid->output = pid->out_min;
    }

    return pid->output;
}

/**
 * @brief  Velocity/Position PI loop for wheel encoder feedback
 */
float PID_VelocityUpdate(pid_controller_t *pid, float current_speed, 
                         float target_speed, float dt)
{
    if (pid == NULL || dt <= 0.0f)
    {
        return 0.0f;
    }

    pid->target = target_speed;
    pid->error  = target_speed - current_speed;

    /* Proportional term */
    float p_out = pid->Kp * pid->error;

    /* Integral term (accumulates position error over time) */
    pid->integral += pid->Ki * pid->error * dt;
    if (pid->integral > pid->integral_limit)
    {
        pid->integral = pid->integral_limit;
    }
    else if (pid->integral < -pid->integral_limit)
    {
        pid->integral = -pid->integral_limit;
    }
    float i_out = pid->integral;

    /* Total velocity loop output (outputs tilt angle offset in degrees) */
    pid->output = p_out + i_out;

    /* Saturate angle adjustment (e.g. limit to +/- 5.0 degrees to prevent tipping over) */
    if (pid->output > pid->out_max)
    {
        pid->output = pid->out_max;
    }
    else if (pid->output < pid->out_min)
    {
        pid->output = pid->out_min;
    }

    return pid->output;
}

