/****************************************************************************
 *
 *   Copyright (C) 2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#pragma once
#include <systemlib/px4_macros.h>


#define BBSRAM_SIZE_FN0 1
#define BBSRAM_SIZE_FN1 256
#define BBSRAM_SIZE_FN2 256
#define BBSRAM_SIZE_FN3 -1

/* The following guides in the amount of the user and interrupt stack
 * data we can save. The amount of storage left will dictate the actual
 * number of entries of the user stack data saved. If it is toobig
 * It will be truncated by the call to stm32_bbsram_savepanic
 */
#define BBSRAM_HEADER_SIZE 20 /* This is an assumption */
#define BBSRAM_USED ((4*BBSRAM_HEADER_SIZE)+(BBSRAM_SIZE_FN0+BBSRAM_SIZE_FN1+BBSRAM_SIZE_FN2))
#define BBSRAM_REAMINING (STM32_BBSRAM_SIZE-BBSRAM_USED)
#if CONFIG_ARCH_INTERRUPTSTACK <= 3
#  define BBSRAM_NUMBER_STACKS 1
#else
#  define BBSRAM_NUMBER_STACKS 2
#endif
#define BBSRAM_FIXED_ELEMENTS_SIZE (sizeof(context_s)+sizeof(info_s))
#define BBSRAM_LEFTOVER (BBSRAM_REAMINING-BBSRAM_FIXED_ELEMENTS_SIZE)

#define CONFIG_ISTACK_SIZE (BBSRAM_LEFTOVER/BBSRAM_NUMBER_STACKS/sizeof(stack_word_t))
#define CONFIG_USTACK_SIZE (BBSRAM_LEFTOVER/BBSRAM_NUMBER_STACKS/sizeof(stack_word_t))

/* The path to the Battery Backed up SRAM */
#define BBSRAM_PATH "/fs/bbr"
/* The sizes of the files to create (-1) use rest of BBSRAM memory */
#define BSRAM_FILE_SIZES { \
    BBSRAM_SIZE_FN0,   /* For Time stamp only */                  \
    BBSRAM_SIZE_FN1,   /* For Current Flight Parameters Copy A */ \
    BBSRAM_SIZE_FN2,   /* For Current Flight Parameters Copy B*/  \
    BBSRAM_SIZE_FN3,   /* For the Panic Log use rest of space */  \
    0                  /* End of table marker */                  \
}

#define MAX_FILE_PATH_LENGTH 40
typedef uint32_t stack_word_t;

typedef  struct {
  int pid;
  struct xcptcontext xcp;                /* Interrupt register save area        */
#if CONFIG_TASK_NAME_SIZE > 0
  char name[CONFIG_TASK_NAME_SIZE+1];    /* Task name (with NUL terminator)     */
#endif
} process_t;

typedef struct {

  uint32_t current_sp;
  uint32_t utopofstack;
  uint32_t ustacksize;
#if CONFIG_ARCH_INTERRUPTSTACK > 3
  uint32_t itopofstack;
  uint32_t istacksize;
#endif

} stack_t;

typedef struct
{

  uint32_t r0;
  uint32_t r1;
  uint32_t r2;
  uint32_t r3;
  uint32_t r4;
  uint32_t r5;
  uint32_t r6;
  uint32_t r7;
  uint32_t r8;
  uint32_t r9;
  uint32_t r10;
  uint32_t r11;
  uint32_t r12;
  uint32_t sp;
  uint32_t lr;
  uint32_t pc;
  uint32_t xpsr;
  uint32_t d0;
  uint32_t d1;
  uint32_t d2;
  uint32_t d3;
  uint32_t d4;
  uint32_t d5;
  uint32_t d6;
  uint32_t d7;
  uint32_t d8;
  uint32_t d9;
  uint32_t d10;
  uint32_t d11;
  uint32_t d12;
  uint32_t d13;
  uint32_t d14;
  uint32_t d15;
  uint32_t fpscr;
  uint32_t sp_main;
  uint32_t sp_process;
  uint32_t apsr;
  uint32_t ipsr;
  uint32_t epsr;
  uint32_t primask;
  uint32_t basepri;
  uint32_t faultmask;
  uint32_t control;
  uint32_t s0;
  uint32_t s1;
  uint32_t s2;
  uint32_t s3;
  uint32_t s4;
  uint32_t s5;
  uint32_t s6;
  uint32_t s7;
  uint32_t s8;
  uint32_t s9;
  uint32_t s10;
  uint32_t s11;
  uint32_t s12;
  uint32_t s13;
  uint32_t s14;
  uint32_t s15;
  uint32_t s16;
  uint32_t s17;
  uint32_t s18;
  uint32_t s19;
  uint32_t s20;
  uint32_t s21;
  uint32_t s22;
  uint32_t s23;
  uint32_t s24;
  uint32_t s25;
  uint32_t s26;
  uint32_t s27;
  uint32_t s28;
  uint32_t s29;
  uint32_t s30;
  uint32_t s31;
} proc_regs_s;

typedef enum {
  ePanicHardFault,
  ePanicAssert,
} panictype_t;

typedef enum {
  eRegs         = 0x01,
  eUserStack    = 0x02,
  eIntStack     = 0x04,
  eStackUnknown = 0x08,
  eStackValid   = eUserStack | eIntStack,
} stuff_t;

typedef struct {
  stuff_t stuff;
  uintptr_t current_regs;
  int lineno;
  char filename[MAX_FILE_PATH_LENGTH];
} info_s;

typedef struct {
  stack_t     stack;
  process_t   proc;
} context_s;

typedef struct {
  info_s    info;
  context_s context;
#if CONFIG_ARCH_INTERRUPTSTACK > 3
  stack_word_t istack[CONFIG_USTACK_SIZE];
#endif
  stack_word_t ustack[CONFIG_ISTACK_SIZE];
} fullcontext_s;


