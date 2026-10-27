/*
****************************************************************************************************************
 * @file       bootloader.h
 * @brief      Bootloader Finite State Machine (FSM) interface and definitions for STM32F1
 * @date       2026/10/25
 * @author     [Gentantun] (nguyenthanhtung8196@gmail.com)
 * @details    Declares states, command types, transition functions, and FSM structures.
 * @ref        stm32f7/Bootloader/boot_process/bootloader.h
 * @copyright  Copyright (c) 2026 RoboTun
****************************************************************************************************************
*/
#ifndef _BOOTLOADER_H_
#define _BOOTLOADER_H_
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
enum bl_state
{
    BL_STATE_IDLE = 0,
    BL_STATE_DWL_START,
    BL_STATE_DWL_DATA,
    BL_STATE_DWL_END,
};

enum bl_cmd_req
{
    BL_CMD_REQ_START,
    BL_CMD_REQ_DATA,
    BL_CMD_REQ_END,
    BL_CMD_REQ_RESET,
};

typedef struct bl_fsm_s bl_fsm_t;

typedef void (*evt_func_t)(bl_fsm_t *state, void *data);

typedef struct
{
    evt_func_t evt_func_start_cmd;
    evt_func_t evt_func_data_cmd;
    evt_func_t evt_func_end_cmd;
    evt_func_t evt_func_reset_cmd;
    evt_func_t evt_func_error;
} bl_fsm_func_t;

struct bl_fsm_s
{
    uint8_t       bl_state;
    bl_fsm_func_t func;
};

typedef bl_fsm_t *bl_fsm_p;
typedef bl_fsm_p (*transition_func_t)(bl_fsm_p state);

/***************************************************************************************************************
**                                         EXTERNAL VARIABLE DECLARATIONS
***************************************************************************************************************/
extern bl_fsm_p main_bl_fsm;

/***************************************************************************************************************
**                                         EXTERNAL FUNCTION PROTOTYPES
***************************************************************************************************************/
void bl_fsm_init(void);
void bl_fsm_changeState(bl_fsm_p state, transition_func_t func);

bl_fsm_p bl_state_idle_transition(bl_fsm_p state);
bl_fsm_p bl_state_start_transition(bl_fsm_p state);
bl_fsm_p bl_state_data_transition(bl_fsm_p state);
bl_fsm_p bl_state_end_transition(bl_fsm_p state);

void bootloader(void);

#ifdef __cplusplus
}
#endif
#endif /* _BOOTLOADER_H_ */
