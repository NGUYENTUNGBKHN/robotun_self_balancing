/**
 ******************************************************************************
 * @file      syscalls.c
 * @brief     Newlib minimal system-call stubs for STM32F1 bare-metal targets
 * @details   Redirects printf() output to debug_uart via _write().
 *            All other syscalls return safe error codes so newlib does not hang.
 *
 * @note      This file is intentionally identical to the stm32f7 version because
 *            the newlib syscall interface is target-independent.
 *
 * Copyright (c) 2020-2024 STMicroelectronics.  All rights reserved.
 * Licensed under the terms found in the LICENSE file.
 ******************************************************************************
 */

/* Includes */
#include <sys/stat.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <sys/times.h>
#include "debug_uart.h"

/* Weak hooks so BSP code can override read/write independently */
extern int __io_putchar(int ch) __attribute__((weak));
extern int __io_getchar(void)   __attribute__((weak));

char *__env[1] = {0};
char **environ = __env;

/* Required by semihosting stubs */
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
    while (1) {}    /* hang — never returns */
}

/**
 * @brief  Read characters from stdin (unimplemented — returns stub input).
 */
__attribute__((weak)) int _read(int file, char *ptr, int len)
{
    (void)file;
    for (int i = 0; i < len; i++)
    {
        *ptr++ = __io_getchar();
    }
    return len;
}

/**
 * @brief  Write characters to stdout/stderr.
 *         Routed to debug_uart_putchar() so printf() appears on USART1.
 */
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

int _unlink(char *name)
{
    (void)name;
    errno = ENOENT;
    return -1;
}

int _times(struct tms *buf)                 { (void)buf; return -1; }

int _stat(char *file, struct stat *st)
{
    (void)file;
    st->st_mode = S_IFCHR;
    return 0;
}

int _link(char *old, char *new)
{
    (void)old; (void)new;
    errno = EMLINK;
    return -1;
}

int _fork(void)                             { errno = EAGAIN; return -1; }

int _execve(char *name, char **argv, char **env)
{
    (void)name; (void)argv; (void)env;
    errno = ENOMEM;
    return -1;
}
