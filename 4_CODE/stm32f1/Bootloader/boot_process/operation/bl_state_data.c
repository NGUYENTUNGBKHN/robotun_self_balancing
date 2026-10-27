/*
****************************************************************************************************************
 * @file       bl_state_data.c
 * @brief      Bootloader DATA state implementation for STM32F1
 * @date       2026/10/25
 * @author     [Gentantun] (nguyenthanhtung8196@gmail.com)
 * @details    Handles stream of firmware payload blocks and transitions to END state.
 * @ref        stm32f7/Bootloader/boot_process/operation/bl_state_data.c
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
static void bl_state_data_evt_data_cmd(bl_fsm_t *state, void *data);
static void bl_state_data_evt_end_cmd(bl_fsm_t *state, void *data);
static void bl_state_data_evt_error(bl_fsm_t *state, void *data);

/***************************************************************************************************************
**                                             FUNCTION DEFINITIONS
***************************************************************************************************************/

static void bl_state_data_evt_data_cmd(bl_fsm_t *state, void *data)
{
    (void)state; (void)data;
    TRACE_INFO("DATA state: block processed\n");
}

static void bl_state_data_evt_end_cmd(bl_fsm_t *state, void *data)
{
    (void)data;
    TRACE_INFO("DATA state: transfer completed, transition to END\n");
    bl_fsm_changeState(state, bl_state_end_transition);
}

static void bl_state_data_evt_error(bl_fsm_t *state, void *data)
{
    (void)data;
    TRACE_INFO("DATA state: write/CRC error, fallback to IDLE\n");
    bl_fsm_changeState(state, bl_state_idle_transition);
}

bl_fsm_p bl_state_data_transition(bl_fsm_p state)
{
    static uint8_t first = 1;
    static bl_fsm_func_t static_func;

    if (first)
    {
        first = 0;
        memset(&static_func, 0, sizeof(static_func));
        static_func.evt_func_data_cmd = bl_state_data_evt_data_cmd;
        static_func.evt_func_end_cmd  = bl_state_data_evt_end_cmd;
        static_func.evt_func_error    = bl_state_data_evt_error;
    }

    state->bl_state = BL_STATE_DWL_DATA;
    state->func = static_func;
    return state;
}

/***************************************************************************************************************
**                                                End of file
***************************************************************************************************************/
