/*
****************************************************************************************************************
 * @file       stepper.h
 * @brief      Stepper motor driver abstraction interface for STM32F1
 * @date       2026/10/25
 * @author     [Gentantun] (nguyenthanhtung8196@gmail.com)
 * @details    Provides directional movement, PWM frequency-based speed control,
 *             finite step counting via timer compare interrupts, and busy status inspection.
 * @ref        stm32f7/Middleware/Stepper/stepper.h
 * @copyright  Copyright (c) 2026 RoboTun
****************************************************************************************************************
*/
#ifndef _STEPPER_H_
#define _STEPPER_H_
#ifdef __cplusplus
extern "C"
{
#endif

/***************************************************************************************************************
**                                                   INCLUDES
***************************************************************************************************************/
#include <stdint.h>

/***************************************************************************************************************
**                                          INTERNAL MACRO DEFINITIONS
***************************************************************************************************************/
typedef enum STEPPER_DIRECTION_TYPE
{
    STEPPER_ERROR   = 0,
    STEPPER_STOP    = 1,
    STEPPER_FORWARD = 2,
    STEPPER_REVERSE = 3
} stepper_direction_type_t;

typedef union STEPPER_DATA_S
{
    struct
    {
        uint8_t                  enable : 1;
        stepper_direction_type_t dir    : 2;
        uint32_t                 speed  : 7;
        uint8_t                  status : 1;
    } bit;

    uint32_t reg;
} stepper_data_t;

typedef struct STEPPER_S
{
    stepper_data_t stepper_reg;
    void    (*stepper_init)(void);
    void    (*stepper_set_direction)(uint8_t dir);
    void    (*stepper_set_frequency)(uint32_t frequency_hz);
    void    (*stepper_start)(void);
    void    (*stepper_stop)(void);
    void    (*stepper_move_steps)(uint32_t steps, uint8_t dir, uint32_t frequency_hz);
    uint8_t (*stepper_is_busy)(void);
} stepper_t;

/***************************************************************************************************************
**                                         EXTERNAL FUNCTION PROTOTYPES
***************************************************************************************************************/
extern stepper_t *stepper_create(void);

#ifdef __cplusplus
}
#endif
#endif /* _STEPPER_H_ */
