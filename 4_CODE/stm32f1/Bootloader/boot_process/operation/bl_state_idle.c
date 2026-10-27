/*
****************************************************************************************************************
 * @file       bl_state_idle.c
 * @brief      Bootloader IDLE state implementation for STM32F1
 * @date       2026/10/25
 * @author     [Gentantun] (nguyenthanhtung8196@gmail.com)
 * @details    Handles START command event in IDLE state and transitions to START state.
 * @ref        stm32f7/Bootloader/boot_process/operation/bl_state_idle.c
 * @copyright  Copyright (c) 2026 RoboTun
****************************************************************************************************************
*/
/***************************************************************************************************************
**                                                   INCLUDES
***************************************************************************************************************/
#include <string.h>
#include "bootloader.h"
#include "common.h"

/***************************************************************************************************************
**                                         INTERNAL FUNCTION PROTOTYPES
***************************************************************************************************************/
static void bl_state_idle_evt_start_cmd(bl_fsm_t *state, void *data);

/***************************************************************************************************************
**                                             FUNCTION DEFINITIONS
***************************************************************************************************************/

static void bl_state_idle_evt_start_cmd(bl_fsm_t *state, void *data)
{
    (void)data;
    TRACE_INFO("IDLE state: START command received\n");
    bl_fsm_changeState(state, bl_state_start_transition);
}

bl_fsm_p bl_state_idle_transition(bl_fsm_p state)
{
    static uint8_t first = 1;
    static bl_fsm_func_t static_func;

    if (first)
    {
        first = 0;
        memset(&static_func, 0, sizeof(static_func));
        static_func.evt_func_start_cmd = bl_state_idle_evt_start_cmd;
    }

    state->bl_state = BL_STATE_IDLE;
    state->func = static_func;
    return state;
}

/***************************************************************************************************************
**                                                End of file
***************************************************************************************************************/
