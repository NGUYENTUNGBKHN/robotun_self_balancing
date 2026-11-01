/*
****************************************************************************************************************
 * @file       bootloader.c
 * @brief      Bootloader execution loop and DFU vector table for STM32F1
 * @date       2026/10/25
 * @author     [Gentantun] (nguyenthanhtung8196@gmail.com)
 * @details    Initialises FSM, enters event polling loop, and defines the .entry_dwl vector table.
 * @ref        stm32f7/Bootloader/boot_process/bootloader.c
 * @copyright  Copyright (c) 2026 RoboTun
****************************************************************************************************************
*/
/***************************************************************************************************************
**                                                   INCLUDES
***************************************************************************************************************/
#include <stdint.h>
#include <stddef.h>
#include "common.h"
#include "bootloader.h"

/***************************************************************************************************************
**                                        INTERNAL VARIABLE DEFINITIONS
***************************************************************************************************************/
static bl_fsm_t g_main_fsm;
bl_fsm_p main_bl_fsm = &g_main_fsm;

/***************************************************************************************************************
**                                         INTERNAL FUNCTION PROTOTYPES
***************************************************************************************************************/
static void bootloader_dwl(uint32_t cmd);
void _entry_download(void);
void _entry_download2(void);

/***************************************************************************************************************
**                                             FUNCTION DEFINITIONS
***************************************************************************************************************/

/* Entry function table placed at DWL_START_ADDRESS (0x08004000) */
void (*const _entry_dwl_func[])(void) __attribute__((section(".entry_dwl"), used)) =
{
    _entry_download,
    _entry_download2
};

/**
 * @brief  Bootloader main processing loop.
 */
void bootloader(void)
{
    uint32_t cmd;

    TRACE_INFO("Enter bootloader FSM loop\n");

    /* Initialise FSM to IDLE state */
    bl_fsm_init();
    cmd = BL_CMD_REQ_START;

    while (1)
    {
        bootloader_dwl(cmd);
    }
}

void _entry_download(void)
{
    TRACE_INFO("Download entry point 1\n");
    while (1)
    {
    }
}

void _entry_download2(void)
{
    TRACE_INFO("Download entry point 2\n");
    while (1)
    {
    }
}

static void bootloader_dwl(uint32_t cmd)
{
    if (main_bl_fsm == NULL)
    {
        return;
    }

    switch (cmd)
    {
    case BL_CMD_REQ_START:
        if (main_bl_fsm->func.evt_func_start_cmd != NULL)
        {
            main_bl_fsm->func.evt_func_start_cmd(main_bl_fsm, NULL);
        }
        break;
    case BL_CMD_REQ_DATA:
        if (main_bl_fsm->func.evt_func_data_cmd != NULL)
        {
            main_bl_fsm->func.evt_func_data_cmd(main_bl_fsm, NULL);
        }
        break;
    case BL_CMD_REQ_END:
        if (main_bl_fsm->func.evt_func_end_cmd != NULL)
        {
            main_bl_fsm->func.evt_func_end_cmd(main_bl_fsm, NULL);
        }
        break;
    case BL_CMD_REQ_RESET:
        if (main_bl_fsm->func.evt_func_reset_cmd != NULL)
        {
            main_bl_fsm->func.evt_func_reset_cmd(main_bl_fsm, NULL);
        }
        break;
    default:
        break;
    }
}

void bl_fsm_init(void)
{
    bl_fsm_changeState(main_bl_fsm, bl_state_idle_transition);
}

void bl_fsm_changeState(bl_fsm_p state, transition_func_t func)
{
    if (func != NULL)
    {
        (void)func(state);
    }
}

/***************************************************************************************************************
**                                                End of file
***************************************************************************************************************/
