/*********************************************************************
*                    SEGGER Microcontroller GmbH                     *
*                        The Embedded Experts                        *
**********************************************************************
*                                                                    *
*            (c) 1995 - 2023 SEGGER Microcontroller GmbH             *
*                                                                    *
*       www.segger.com     Support: support@segger.com               *
*                                                                    *
**********************************************************************
*                                                                    *
*       SEGGER SystemView * Real-time application analysis           *
*                                                                    *
**********************************************************************
*                                                                    *
* All rights reserved.                                               *
*                                                                    *
* SEGGER strongly recommends to not make any changes                 *
* to or modify the source code of this software in order to stay     *
* compatible with the SystemView and RTT protocol, and J-Link.       *
*                                                                    *
* Redistribution and use in source and binary forms, with or         *
* without modification, are permitted provided that the following    *
* condition is met:                                                  *
*                                                                    *
* o Redistributions of source code must retain the above copyright   *
*   notice, this condition and the following disclaimer.             *
*                                                                    *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND             *
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,        *
* INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF           *
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE           *
* DISCLAIMED. IN NO EVENT SHALL SEGGER Microcontroller BE LIABLE FOR *
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR           *
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT  *
* OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;    *
* OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF      *
* LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT          *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE  *
* USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH   *
* DAMAGE.                                                            *
*                                                                    *
**********************************************************************
*                                                                    *
*       SystemView version: 3.50                                    *
*                                                                    *
**********************************************************************
-------------------------- END-OF-HEADER -----------------------------

File    : SEGGER_SYSVIEW_Conf.h
Purpose : SEGGER SystemView configuration file.
          Set defines which deviate from the defaults (see SEGGER_SYSVIEW_ConfDefaults.h) here.          
Revision: $Rev: 21292 $

Additional information:
  Required defines which must be set are:
    SEGGER_SYSVIEW_GET_TIMESTAMP
    SEGGER_SYSVIEW_GET_INTERRUPT_ID
  For known compilers and cores, these might be set to good defaults
  in SEGGER_SYSVIEW_ConfDefaults.h.
  
  SystemView needs a (nestable) locking mechanism.
  If not defined, the RTT locking mechanism is used,
  which then needs to be properly configured.
*/

#ifndef SEGGER_SYSVIEW_CONF_H
#define SEGGER_SYSVIEW_CONF_H

/*********************************************************************
*
*       Defines, configurable
*
**********************************************************************
*/

/*********************************************************************
* TODO: Add your defines here.                                       *
**********************************************************************
*/
#include "main.h" // 包含HAL库头文件，以便使用 SystemCoreClock

// 告诉 SystemView 你的芯片跑多快
// 方式 A：直接写死（比如 400MHz，写成 400000000）
// #define SEGGER_SYSVIEW_CORE_FREQ      400000000

// 方式 B：使用 HAL 库的全局变量（推荐，更灵活）
// extern uint32_t SystemCoreClock;
#define SEGGER_SYSVIEW_CORE_FREQ      SystemCoreClock

// 告诉 SystemView 系统时间戳的频率（通常等于 CPU 主频）
#define SEGGER_SYSVIEW_TIMESTAMP_FREQ SEGGER_SYSVIEW_CORE_FREQ

// (可选) 为了避免 STM32H7 的 Cache 问题，建议把 RTT Buffer 放到特定区域
// 如果你不知道怎么改链接脚本，先暂时不用管这一行
/* ==================================================================== */
/* 强制将 SystemView 的内部控制块也放到非 Cache 区域                      */
/* ==================================================================== */
#define SEGGER_SYSVIEW_SECTION        ".rtt_memory"

/* >>>>> 新增：禁止 SystemView 使用底层汇编优化函数，强制回退到 C 语言版本 <<<<< */
#define SEGGER_SYSVIEW_USE_RTT_ASM    0
#define SEGGER_SYSVIEW_RECORD_SYSTICK  0
#endif  // SEGGER_SYSVIEW_CONF_H

/*************************** End of file ****************************/
