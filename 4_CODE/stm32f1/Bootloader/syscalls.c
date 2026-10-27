/*
****************************************************************************************************************
 * @file       syscalls.c
 * @brief      Minimal Newlib system-call stubs for STM32F1 Bootloader
 * @date       2026/10/25
 * @author     [Gentantun] (nguyenthanhtung8196@gmail.com)
 * @details    Routes printf() and puts() output directly to debug_uart_putchar().
 * @ref        stm32f7/Bootloader/syscalls.c
 * @copyright  Copyright (c) 2026 RoboTun
****************************************************************************************************************
*/
/***************************************************************************************************************
**                                                   INCLUDES
***************************************************************************************************************/
#include <sys/stat.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <sys/times.h>
#include "debug_uart.h"

/***************************************************************************************************************
**                                        EXTERNAL VARIABLE DECLARATIONS
***************************************************************************************************************/
char *__env[1] = {0};
char **environ = __env;

/***************************************************************************************************************
**                                             FUNCTION DEFINITIONS
***************************************************************************************************************/
void initialise_monitor_handles(void) {}

int _getpid(void)                           { return 1; }

int _kill(int pid, int sig)
{
    (void)pid; (void)sig;
    errno = EINVAL;
    return -1;
}

void _exit(int status)
{
    _kill(status, -1);
    while (1) {}
}

__attribute__((weak)) int _read(int file, char *ptr, int len)
{
    (void)file; (void)ptr;
    return len;
}

__attribute__((weak)) int _write(int file, char *ptr, int len)
{
    (void)file;
    for (int i = 0; i < len; i++)
    {
        debug_uart_putchar(*ptr++);
    }
    return len;
}

int _close(int file)                        { (void)file; return -1; }

int _fstat(int file, struct stat *st)
{
    (void)file;
    st->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int file)                       { (void)file; return 1; }

int _lseek(int file, int ptr, int dir)
{
    (void)file; (void)ptr; (void)dir;
    return 0;
}

int _open(char *path, int flags, ...)       { (void)path; (void)flags; return -1; }

int _wait(int *status)
{
    (void)status;
    errno = ECHILD;
    return -1;
}

int _unlink(char *name)                     { (void)name; errno = ENOENT; return -1; }

int _times(struct tms *buf)                 { (void)buf; return -1; }

int _stat(char *file, struct stat *st)
{
    (void)file;
    st->st_mode = S_IFCHR;
    return 0;
}

int _link(char *old, char *new)             { (void)old; (void)new; errno = EMLINK; return -1; }

int _fork(void)                             { errno = EAGAIN; return -1; }

int _execve(char *name, char **argv, char **env)
{
    (void)name; (void)argv; (void)env;
    errno = ENOMEM;
    return -1;
}

/***************************************************************************************************************
**                                                End of file
***************************************************************************************************************/
