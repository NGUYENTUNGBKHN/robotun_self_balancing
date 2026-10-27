/*
****************************************************************************************************************
 * @file       bl_state_start.c
 * @brief      Bootloader START state implementation for STM32F1
 * @date       2026/10/25
 * @author     [Gentantun] (nguyenthanhtung8196@gmail.com)
 * @details    Handles download handshake and transitions to DATA state.
 * @ref        stm32f7/Bootloader/boot_process/operation/bl_state_start.c
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
static void bl_state_start_evt_start_cmd(bl_fsm_t *state, void *data);
static void bl_state_start_evt_data_cmd(bl_fsm_t *state, void *data);
static void bl_state_start_evt_error(bl_fsm_t *state, void *data);

/***************************************************************************************************************
**                                             FUNCTION DEFINITIONS
***************************************************************************************************************/

static void bl_state_start_evt_start_cmd(bl_fsm_t *state, void *data)
{
    (void)state; (void)data;
    TRACE_INFO("START state: received start request\n");
}

static void bl_state_start_evt_data_cmd(bl_fsm_t *state, void *data)
{
    (void)data;
    TRACE_INFO("START state: received data packet, transition to DATA\n");
    bl_fsm_changeState(state, bl_state_data_transition);
}

static void bl_state_start_evt_error(bl_fsm_t *state, void *data)
{
    (void)data;
    TRACE_INFO("START state: error encountered, fallback to IDLE\n");
    bl_fsm_changeState(state, bl_state_idle_transition);
}

bl_fsm_p bl_state_start_transition(bl_fsm_p state)
{
    static uint8_t first = 1;
    static bl_fsm_func_t static_func;

    if (first)
    {
        first = 0;
        memset(&static_func, 0, sizeof(static_func));
        static_func.evt_func_start_cmd = bl_state_start_evt_start_cmd;
        static_func.evt_func_data_cmd  = bl_state_start_evt_data_cmd;
        static_func.evt_func_error     = bl_state_start_evt_error;
    }

    state->bl_state = BL_STATE_DWL_START;
    state->func = static_func;
    return state;
}

/***************************************************************************************************************
**                                                End of file
***************************************************************************************************************/
