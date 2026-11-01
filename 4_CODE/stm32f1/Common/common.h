/*
****************************************************************************************************************
 * @file       common.h
 * @brief      
 * @date       2026/10/24
 * @author     [Gentantun] (nguyenthanhtung8196@gmail.com)
 * @details    
 * @ref        
 * @copyright  Copyright (c) 2026 RoboTun
****************************************************************************************************************
*/
#ifndef _COMMON_H_
#define _COMMON_H_
#ifdef __cplusplus
extern "C"
{
#endif

/* CODE */
/* CODE */
#include <stdio.h>
#include "stm32f1xx.h"

typedef enum Code_Status_S
{
    ACE_SUCCESS = 0,
    ACE_ERROR
}code_status_t;

#define DEBUG
#if defined(DEBUG)
/* TRACE info : normal log */
#define TRACE_INFO(format, ... )                                                        \
    printf("%s (%d) : " format, __FUNCTION__, __LINE__, ##__VA_ARGS__)                  \
/* TRACE info : normal log */
#define ERROR(format, ... )                                                              \
    printf("%s (%d) : " format, __FUNCTION__, __LINE__, ##__VA_ARGS__)                  \

#else
#define TRACE_INFO(__format__, ...)
#define ERROR(__format__, ...)
#endif //  DEBUG

#ifdef __cplusplus
}
#endif
#endif

