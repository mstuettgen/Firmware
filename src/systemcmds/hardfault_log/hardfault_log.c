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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
__EXPORT int hardfault_log_main(int argc, char *argv[]);

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static int genfault(int fault);
/****************************************************************************
 * Private Data
 ****************************************************************************/
/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * genfault
 ****************************************************************************/

static int genfault(int fault)
{

  /* Pointer to System Control Block's  System Control Register */

  uint32_t *pCCR = (uint32_t *)0xE000ED14;

  static volatile int k = 0;

  switch(fault)
  {
  case 0:

    /* Enable divide by 0 fault generation */

    *pCCR |= 0x10;

    k =  1 / fault;

    /* This is not going to happen
     * Enable divide by 0 fault generation
     */

    *pCCR &= ~0x10;
    break;

  case 1:
      ASSERT( fault== 0);
      /* This is not going to happen */
      break;

  case 2:
      printf("null %s\n",NULL);
      /* This is not going to happen */
      break;

  case 3:
    {
      char marker[20];
      strncpy(marker, "<-- ", sizeof(marker));
      printf("nill %s\n","");
      printf("nill fault==3 %s\n",(fault==3) ? "3" : "");
      printf("nill fault!=3 %s\n",(fault!=3) ? "3" : "");
      printf("0x%08x 0x%08x%s\n", fault, -fault, (fault==3) ? "" : marker);
      printf("0x%08x 0x%08x%s\n", fault, -fault, (fault!=3) ? "" : marker);
      printf("0x%08x 0x%08x%s\n", fault, -fault, fault==3 ? "" : marker);
      printf("0x%08x 0x%08x%s\n", fault, -fault, fault!=3 ? "" : marker);
    }
      /* This is not going to happen */
      break;

  default:
    break;

  }
  UNUSED(k);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: test_os_timing
 ****************************************************************************/

int hardfault_log_main(int argc, char *argv[])
{

        /*
         * Generate a fault
         */
        if (!strcmp(argv[1], "fault")) {
            int fault = 0;
            if (argc > 2) {
                fault = atol(argv[2]);
            }
            return genfault(fault);
        }

        fprintf(stderr, "unrecognised command, try 'fault' ,'parse' or 'commit'\n");
        return -EINVAL;
}
