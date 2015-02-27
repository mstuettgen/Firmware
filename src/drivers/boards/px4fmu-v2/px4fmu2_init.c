/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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

/**
 * @file px4fmu2_init.c
 *
 * PX4FMUv2-specific early startup code.  This file implements the
 * nsh_archinitialize() function that is called early by nsh during startup.
 *
 * Code here is run before the rcS script is invoked; it should start required
 * subsystems and perform board-specific initialization.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <debug.h>
#include <errno.h>
#include <fcntl.h>


#include <nuttx/arch.h>
#include <nuttx/spi/spi.h>
#include <nuttx/i2c.h>
#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>
#include <nuttx/analog/adc.h>
#include <nuttx/mm/gran.h>

#include <stm32.h>
#include "board_config.h"
#include <stm32_uart.h>
#include <stm32_bbsram.h>

#include <arch/board/board.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_led.h>

#include <systemlib/px4_macros.h>
#include <systemlib/cpuload.h>
#include <systemlib/perf_counter.h>

#include <systemlib/hardfault_log.h>
#include <lib/version/version.h>

#if defined(CONFIG_HAVE_CXX) && defined(CONFIG_HAVE_CXXINITIALIZE)
#include <systemlib/systemlib.h>
#endif

#include "up_internal.h"

/* todo: This is constant but not proper */
__BEGIN_DECLS
extern void led_off(int led);
__END_DECLS

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
#define FREEZE_STR(s) #s
#define STRINGIFY(s) FREEZE_STR(s)
#define HARDFAULT_FILENO 3
#define HARDFAULT_PATH BBSRAM_PATH""STRINGIFY(HARDFAULT_FILENO)

/* Configuration ************************************************************/

/****************************************************************************
 * Protected Functions
 ****************************************************************************/

#if defined(CONFIG_FAT_DMAMEMORY)
# if !defined(CONFIG_GRAN) || !defined(CONFIG_FAT_DMAMEMORY)
#  error microSD DMA support requires CONFIG_GRAN
# endif

#ifdef CONFIG_FAT_DMAMEMORY

static GRAN_HANDLE dma_allocator;

/*
 * The DMA heap size constrains the total number of things that can be
 * ready to do DMA at a time.
 *
 * For example, FAT DMA depends on one sector-sized buffer per filesystem plus
 * one sector-sized buffer per file.
 *
 * We use a fundamental alignment / granule size of 64B; this is sufficient
 * to guarantee alignment for the largest STM32 DMA burst (16 beats x 32bits).
 */
static uint8_t g_dma_heap[8192] __attribute__((aligned(64)));
static perf_counter_t g_dma_perf;
#endif

static void
dma_alloc_init(void)
{
	dma_allocator = gran_initialize(g_dma_heap,
					sizeof(g_dma_heap),
					7,  /* 128B granule - must be > alignment (XXX bug?) */
					6); /* 64B alignment */

	if (dma_allocator == NULL) {
		syslog(LOG_ERR, "[boot] DMA allocator setup FAILED");

	} else {
		g_dma_perf = perf_alloc(PC_COUNT, "DMA allocations");
	}
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/*
 * DMA-aware allocator stubs for the FAT filesystem.
 */

__EXPORT void *fat_dma_alloc(size_t size);
__EXPORT void fat_dma_free(FAR void *memory, size_t size);

void *
fat_dma_alloc(size_t size)
{
	perf_count(g_dma_perf);
	return gran_alloc(dma_allocator, size);
}

void
fat_dma_free(FAR void *memory, size_t size)
{
	gran_free(dma_allocator, memory, size);
}

#else

# define dma_alloc_init()

#endif

/************************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This entry point
 *   is called early in the intitialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

__EXPORT void
stm32_boardinitialize(void)
{
	/* configure SPI interfaces */
	stm32_spiinitialize();

	/* configure LEDs */
	board_led_initialize();
}

/****************************************************************************
 * Name: nsh_archinitialize
 *
 * Description:
 *   Perform architecture specific initialization
 *
 ****************************************************************************/

static struct spi_dev_s *spi1;
static struct spi_dev_s *spi2;
static struct spi_dev_s *spi4;
static struct sdio_dev_s *sdio;

#include <math.h>

#ifdef __cplusplus
__EXPORT int matherr(struct __exception *e)
{
	return 1;
}
#else
__EXPORT int matherr(struct exception *e)
{
	return 1;
}
#endif

__EXPORT int nsh_archinitialize(void)
{

	/* configure ADC pins */
	stm32_configgpio(GPIO_ADC1_IN2);	/* BATT_VOLTAGE_SENS */
	stm32_configgpio(GPIO_ADC1_IN3);	/* BATT_CURRENT_SENS */
	stm32_configgpio(GPIO_ADC1_IN4);	/* VDD_5V_SENS */
	stm32_configgpio(GPIO_ADC1_IN13);	/* FMU_AUX_ADC_1 */
	stm32_configgpio(GPIO_ADC1_IN14);	/* FMU_AUX_ADC_2 */
	stm32_configgpio(GPIO_ADC1_IN15);	/* PRESSURE_SENS */

	/* configure power supply control/sense pins */
	stm32_configgpio(GPIO_VDD_5V_PERIPH_EN);
	stm32_configgpio(GPIO_VDD_3V3_SENSORS_EN);
	stm32_configgpio(GPIO_VDD_BRICK_VALID);
	stm32_configgpio(GPIO_VDD_SERVO_VALID);
	stm32_configgpio(GPIO_VDD_5V_HIPOWER_OC);
	stm32_configgpio(GPIO_VDD_5V_PERIPH_OC);

#if defined(CONFIG_HAVE_CXX) && defined(CONFIG_HAVE_CXXINITIALIZE)

	/* run C++ ctors before we go any further */

	up_cxxinitialize();

#	if defined(CONFIG_EXAMPLES_NSH_CXXINITIALIZE)
#  		error CONFIG_EXAMPLES_NSH_CXXINITIALIZE Must not be defined! Use CONFIG_HAVE_CXX and CONFIG_HAVE_CXXINITIALIZE.
#	endif

#else
#  error platform is dependent on c++ both CONFIG_HAVE_CXX and CONFIG_HAVE_CXXINITIALIZE must be defined.
#endif
#if defined(CONFIG_STM32_BBSRAM)
	int filesizes[CONFIG_STM32_BBSRAM_FILES+1] = BSRAM_FILE_SIZES;
	int nfc = stm32_bbsraminitialize(BBSRAM_PATH, filesizes);

        syslog(LOG_INFO, "[boot] %d Battery Back File(s) \n",nfc);
#if defined(CONFIG_STM32_SAVE_CRASHDUMP)
        int fd = open(HARDFAULT_PATH,O_RDONLY);
        if (fd < 0 ) {
            syslog(LOG_INFO, "[boot] Failed to open Crash Log file (%d)\n",fd);
        } else {
            struct bbsramd_s desc;
            int rv = ioctl(fd, STM32_BBSRAM_GETDESC_IOCTL, (unsigned long)((uintptr_t)&desc));
            if (rv < 0) {
                syslog(LOG_INFO, "[boot] Failed to get Crash Log dec (%d)\n",rv);
            } else {
                syslog(LOG_INFO, "[boot] Crash Log info File No %d Length %d state:0x%02x %lu sec, %lu nsec)\n",
                    (unsigned int)desc.fileno, (unsigned int) desc.len, (unsigned int)desc.flags,
                    (unsigned long)desc.lastwrite.tv_sec,(unsigned long)desc.lastwrite.tv_nsec);
                if (desc.lastwrite.tv_sec || desc.lastwrite.tv_nsec) {
                    syslog(LOG_INFO, "[boot] Crash Log Valid...parsing)\n");

                }
                rv = close(fd);
                if (rv < 0) {
                    syslog(LOG_INFO, "[boot] Failed to Close Crash Log (%d)\n",rv);
                } else {
                    rv = unlink(HARDFAULT_PATH);
                    if (rv < 0) {
                        syslog(LOG_INFO, "[boot] Failed to Rearm Crash Log (%d)\n",rv);
                    }
                }
            }
        }
#endif // CONFIG_STM32_SAVE_CRASHDUMP
#endif // CONFIG_STM32_BBSRAM

	/* configure the high-resolution time/callout interface */
	hrt_init();

	/* configure the DMA allocator */
	dma_alloc_init();

	/* configure CPU load estimation */
#ifdef CONFIG_SCHED_INSTRUMENTATION
	cpuload_initialize_once();
#endif

	/* set up the serial DMA polling */
	static struct hrt_call serial_dma_call;
	struct timespec ts;

	/*
	 * Poll at 1ms intervals for received bytes that have not triggered
	 * a DMA event.
	 */
	ts.tv_sec = 0;
	ts.tv_nsec = 1000000;

	hrt_call_every(&serial_dma_call,
		       ts_to_abstime(&ts),
		       ts_to_abstime(&ts),
		       (hrt_callout)stm32_serial_dma_poll,
		       NULL);

	/* initial LED state */
	drv_led_start();
	led_off(LED_AMBER);

	/* Configure SPI-based devices */

	spi1 = up_spiinitialize(1);

	if (!spi1) {
		syslog(LOG_ERR, "[boot] FAILED to initialize SPI port 1\n");
		board_led_on(LED_AMBER);
		return -ENODEV;
	}

	/* Default SPI1 to 1MHz and de-assert the known chip selects. */
	SPI_SETFREQUENCY(spi1, 10000000);
	SPI_SETBITS(spi1, 8);
	SPI_SETMODE(spi1, SPIDEV_MODE3);
	SPI_SELECT(spi1, PX4_SPIDEV_GYRO, false);
	SPI_SELECT(spi1, PX4_SPIDEV_ACCEL_MAG, false);
	SPI_SELECT(spi1, PX4_SPIDEV_BARO, false);
	SPI_SELECT(spi1, PX4_SPIDEV_MPU, false);
	up_udelay(20);

	syslog(LOG_INFO, "[boot] Initialized SPI port 1 (SENSORS)\n");

	/* Get the SPI port for the FRAM */

	spi2 = up_spiinitialize(2);

	if (!spi2) {
		syslog(LOG_ERR, "[boot] FAILED to initialize SPI port 2\n");
		board_led_on(LED_AMBER);
		return -ENODEV;
	}

	/* Default SPI2 to 37.5 MHz (40 MHz rounded to nearest valid divider, F4 max)
	 * and de-assert the known chip selects. */

	// XXX start with 10.4 MHz in FRAM usage and go up to 37.5 once validated
	SPI_SETFREQUENCY(spi2, 12 * 1000 * 1000);
	SPI_SETBITS(spi2, 8);
	SPI_SETMODE(spi2, SPIDEV_MODE3);
	SPI_SELECT(spi2, SPIDEV_FLASH, false);

	syslog(LOG_INFO, "[boot] Initialized SPI port 2 (RAMTRON FRAM)\n");

	spi4 = up_spiinitialize(4);

	/* Default SPI4 to 1MHz and de-assert the known chip selects. */
	SPI_SETFREQUENCY(spi4, 10000000);
	SPI_SETBITS(spi4, 8);
	SPI_SETMODE(spi4, SPIDEV_MODE3);
	SPI_SELECT(spi4, PX4_SPIDEV_EXT0, false);
	SPI_SELECT(spi4, PX4_SPIDEV_EXT1, false);

	syslog(LOG_INFO, "[boot] Initialized SPI port 4\n");

#ifdef CONFIG_MMCSD
	/* First, get an instance of the SDIO interface */

	sdio = sdio_initialize(CONFIG_NSH_MMCSDSLOTNO);

	if (!sdio) {
		syslog(LOG_ERR, "[boot] Failed to initialize SDIO slot %d\n",
		       CONFIG_NSH_MMCSDSLOTNO);
		return -ENODEV;
	}

	/* Now bind the SDIO interface to the MMC/SD driver */
	int ret = mmcsd_slotinitialize(CONFIG_NSH_MMCSDMINOR, sdio);

	if (ret != OK) {
		syslog(LOG_ERR, "[boot] Failed to bind SDIO to the MMC/SD driver: %d\n", ret);
		return ret;
	}

	/* Then let's guess and say that there is a card in the slot. There is no card detect GPIO. */
	sdio_mediachange(sdio, true);

	syslog(LOG_INFO, "[boot] Initialized SDIO\n");
#endif

	return OK;
}

static void print_stack_info(int size, uint32_t topaddr, uint32_t spaddr,
                             uint32_t botaddr, char *sp_name, char *buffer, int max, int fd)
{

  int n = 0;
  n =   snprintf(&buffer[n], max-n, "%s stack:\n",sp_name);
  n +=  snprintf(&buffer[n], max-n, "  top:    0x%08x\n", topaddr);
  n +=  snprintf(&buffer[n], max-n, "  sp:     0x%08x\n", spaddr);
  write(fd, buffer,n);
  n = 0;
  n +=  snprintf(&buffer[n], max-n, "  bottom: 0x%08x\n", botaddr);
  n +=  snprintf(&buffer[n], max-n, "  size:   0x%08x\n", size);
  write(fd, buffer,n);
#ifndef CONFIG_STACK_COLORATION
  FAR struct tcb_s tcb;
  tcb.stack_alloc_ptr = (void*) botaddr;
  tcb.adj_stack_size = size;
  n = snprintf(buffer, max,         "  used:   %08x\n", up_check_tcbstack(&tcb));
  write(fd, buffer,n);
#endif
}

static void print_stack(stack_word_t *swindow, int winsize, uint32_t wtopaddr,
                        uint32_t topaddr, uint32_t spaddr, uint32_t botaddr,
                        char *sp_name, char *buffer, int max, int fd)
{
   char marker[30];
   for (int i = winsize; i >= 0; i--) {
       if (wtopaddr == topaddr) {
           strncpy(marker, "<-- ", sizeof(marker));
           strncat(marker, sp_name, sizeof(marker));
           strncat(marker, " top", sizeof(marker));
       } else if (wtopaddr == spaddr) {
           strncpy(marker, "<-- ", sizeof(marker));
           strncat(marker, sp_name, sizeof(marker));
       } else if (wtopaddr == botaddr) {
           strncpy(marker, "<-- ", sizeof(marker));
           strncat(marker, sp_name, sizeof(marker));
           strncat(marker, " bottom", sizeof(marker));
       } else {
           marker[0] = '\0';
       }
       int n = snprintf(buffer, max,"0x%08x 0x%08x%s\n", wtopaddr, swindow[i], marker);
       write(fd, buffer,n);
       wtopaddr--;
    }
}

static void print_register(fullcontext_s* fc, char *buffer, int max, int fd)
{
    int n = snprintf(buffer, max, "r0:0x%08x r1:0x%08x  r2:0x%08x  r3:0x%08x  r4:0x%08x  r5:0x%08x r6:0x%08x r7:0x%08x\n",
                 fc->context.proc.xcp.regs[REG_R0],  fc->context.proc.xcp.regs[REG_R1],
                 fc->context.proc.xcp.regs[REG_R2],  fc->context.proc.xcp.regs[REG_R3],
                 fc->context.proc.xcp.regs[REG_R4],  fc->context.proc.xcp.regs[REG_R5],
                 fc->context.proc.xcp.regs[REG_R6],  fc->context.proc.xcp.regs[REG_R7]);

    write(fd, buffer,n);
    n  = snprintf(buffer, max, "r8:0x%08x r9:0x%08x r10:0x%08x r11:0x%08x r12:0x%08x  sp:0x%08x lr:0x%08x pc:0x%08x\n",
                  fc->context.proc.xcp.regs[REG_R8],  fc->context.proc.xcp.regs[REG_R9],
                  fc->context.proc.xcp.regs[REG_R10], fc->context.proc.xcp.regs[REG_R11],
                  fc->context.proc.xcp.regs[REG_R12], fc->context.proc.xcp.regs[REG_R13],
                  fc->context.proc.xcp.regs[REG_R14], fc->context.proc.xcp.regs[REG_R15]);

    write(fd, buffer,n);

#ifdef CONFIG_ARMV7M_USEBASEPRI
    n = snprintf(buffer, max, "xpsr:0x%08x basepri:0x%08x control:0x%08x\n",
                 fc->context.proc.xcp.regs[REG_XPSR],  fc->context.proc.xcp.regs[REG_BASEPRI],
                 getcontrol());
#else
    n = snprintf(buffer, max, "xpsr:0x%08x primask:0x%08x control:0x%08x\n",
                 fc->context.proc.xcp.regs[REG_XPSR],  fc->context.proc.xcp.regs[REG_PRIMASK],
                 getcontrol());
#endif
    write(fd, buffer,n);

#ifdef REG_EXC_RETURN
    n = snprintf(buffer, max, "exe return:0x%08x\n", fc->context.proc.xcp.regs[REG_EXC_RETURN]);
    write(fd, buffer,n);
#endif
}


fullcontext_s dump;

__EXPORT void board_crashdump(uint32_t currentsp, void *tcb, uint8_t *filename, int lineno)
{
  (void)irqsave();

  struct tcb_s *rtcb = (struct tcb_s *)tcb;

  /* Zero out everything */

  memset(&dump,0,sizeof(dump));

  /* Save Info */

  dump.info.lineno = lineno;

  if (filename) {

    int offset = 0;
    unsigned int len = strlen((char*)filename) + 1;
    if (len > sizeof(dump.info.filename)) {
        offset = len - sizeof(dump.info.filename) ;
    }
    strncpy(dump.info.filename, (char*)&filename[offset], sizeof(dump.info.filename));
  }

  /* Save the value of the pointer for current_regs as debugging info.
   * It should be NULL in case of an ASSERT and will aid in cross
   * checking the validity of system memory at the time of the
   * fault.
   */

  dump.info.current_regs = (uintptr_t) current_regs;

  /* Save Context */

  /* If not NULL then we are in an interrupt context and the user context
   * is in current_regs else we are running in the users context
   */

#if CONFIG_TASK_NAME_SIZE > 0
  strncpy(dump.context.proc.name, rtcb->name, CONFIG_TASK_NAME_SIZE);
#endif

  dump.context.proc.pid = rtcb->pid;

  dump.context.stack.current_sp = currentsp;

  if (current_regs)
    {
      dump.info.stuff |= eRegs;
      memcpy(&dump.context.proc.xcp.regs, (void*)current_regs, sizeof(dump.context.proc.xcp));
      currentsp = dump.context.proc.xcp.regs[REG_R13];
    }


  dump.context.stack.itopofstack = (uint32_t) &g_intstackbase;;
  dump.context.stack.istacksize = (CONFIG_ARCH_INTERRUPTSTACK & ~3);

  if (dump.context.proc.pid == 0) {

      dump.context.stack.utopofstack = g_idle_topstack - 4;
      dump.context.stack.ustacksize = CONFIG_IDLETHREAD_STACKSIZE;

  } else {
      dump.context.stack.utopofstack = (uint32_t) rtcb->adj_stack_ptr;
      dump.context.stack.ustacksize = (uint32_t) rtcb->adj_stack_size;;
  }

#if CONFIG_ARCH_INTERRUPTSTACK > 3

  /* Get the limits on the interrupt stack memory */

  dump.context.stack.itopofstack = (uint32_t)&g_intstackbase;
  dump.context.stack.istacksize  = (CONFIG_ARCH_INTERRUPTSTACK & ~3);

  /* If the current stack pointer is within the interrupt stack then
   * save the interrupt stack data centered about the interrupt stack pointer
   */

  if (dump.context.stack.current_sp <= dump.context.stack.itopofstack &&
      dump.context.stack.current_sp > dump.context.stack.itopofstack - dump.context.stack.istacksize)
    {
      dump.info.stuff |= eIntStack;
      memcpy(&dump.istack, (void *)(dump.context.stack.current_sp-sizeof(dump.istack)/2),
             sizeof(dump.istack));
   }

#endif

  /*  If the saved context of the interrupted process's stack pointer lies within the
   * allocated user stack memory then save the user stack centered about the user sp
   */
  if (currentsp <= dump.context.stack.utopofstack &&
      currentsp  > dump.context.stack.utopofstack - dump.context.stack.ustacksize)
    {
      dump.info.stuff |= eUserStack;
      memcpy(&dump.ustack, (void *)(currentsp-sizeof(dump.ustack)/2), sizeof(dump.ustack));
    }

  /* Oh boy we have a real hot mess on our hands so save above and below the
   * current sp
   */

  if ((dump.info.stuff & eStackValid) == 0)
    {
      dump.info.stuff |= eStackUnknown;
#if CONFIG_ARCH_INTERRUPTSTACK > 3
      /* sp and above in istack */
      memcpy(&dump.istack, (void *)dump.context.stack.current_sp, sizeof(dump.istack));
      /* below in ustack */
      memcpy(&dump.ustack, (void *)(dump.context.stack.current_sp-sizeof(dump.ustack)),
             sizeof(dump.ustack));
#else
      /* save above and below in ustack */
      memcpy(&dump.ustack, (void *)(dump.context.stack.current_sp-sizeof(dump.ustack)/2),
             sizeof(dump.ustack)/2);
#endif
    }

  stm32_bbsram_savepanic(HARDFAULT_FILENO, (uint8_t*)&dump, sizeof(dump));

/* This it a test */
  char line[200];
  int fd = fileno(stdout);
  int n;

  int spaddr = dump.context.stack.current_sp;
  int topaddr = dump.context.stack.itopofstack;
  int botaddr = dump.context.stack.itopofstack - dump.context.stack.istacksize;
  int winsize = arraySize(dump.istack);
  int wtopaddr = spaddr + winsize/2;
  if ((dump.info.stuff & eIntStack) != 0) {

      print_stack(dump.istack, winsize, wtopaddr, topaddr, spaddr, botaddr,
                  "Interrupt sp", line, sizeof(line), fd);
  }
  if ((dump.info.stuff & eUserStack) != 0) {
      spaddr = dump.context.proc.xcp.regs[REG_R13];
      topaddr = dump.context.stack.utopofstack;
      botaddr = dump.context.stack.utopofstack - dump.context.stack.ustacksize;
      winsize = arraySize(dump.ustack);
      wtopaddr = spaddr + winsize/2;

      print_stack(dump.istack, winsize, wtopaddr, topaddr, spaddr, botaddr,
                  "User sp", line, sizeof(line), fd);
  }

  bool isFault = dump.info.current_regs != 0 && dump.context.proc.pid == 0;
  if (isFault) {
      n = snprintf(line, sizeof(line), "System fault:\n Type:Hard Fault\n");
  } else {
      n = snprintf(line, sizeof(line), "System fault:\n Type:Assertion failed ");
  }
  write(fd, line,n);

#ifdef CONFIG_PRINT_TASKNAME
    n = snprintf(line, sizeof(line), " in file:%s at line: %d running task: %s\n",dump.info.filename, dump.info.lineno, dump.proc.name);
#else
    n = snprintf(line, sizeof(line), " in file:%s at line: %d \n", dump.info.filename, dump.info.lineno);
#endif
    write(fd, line,n);

    n = snprintf(line, sizeof(line), "FW git-hash: %s\n", FW_GIT);
    write(fd, line,n);
    n = snprintf(line, sizeof(line), "Build datetime: %s %s\n", __DATE__, __TIME__);
    write(fd, line,n);

  if (dump.info.stuff & eRegs) {
      n = snprintf(line, sizeof(line), "Processor registers: from 0x%08x\n", dump.info.current_regs);
      write(fd, line,n);
    print_register(&dump,line, sizeof(line), fd);
  }


  n = snprintf(line, sizeof(line), "System Stacks\n");
  if ((dump.info.stuff & eIntStack) != 0) {

      spaddr = dump.context.stack.current_sp;
      topaddr = dump.context.stack.itopofstack;
      botaddr = dump.context.stack.itopofstack - dump.context.stack.istacksize;

      print_stack_info(dump.context.stack.istacksize, topaddr, spaddr,
                       botaddr, "IRQ", line, sizeof(line), fd);
  }

  if ((dump.info.stuff & eUserStack) != 0) {

      spaddr = dump.context.proc.xcp.regs[REG_R13];
      topaddr = dump.context.stack.utopofstack;
      botaddr = dump.context.stack.utopofstack - dump.context.stack.ustacksize;

      print_stack_info(dump.context.stack.ustacksize, topaddr, spaddr,
                       botaddr, "IRQ", line, sizeof(line), fd);
  }

  n = snprintf(line, sizeof(line), "\n Oh I see the Problem...");
  write(fd, line,n);
  sleep(1);
  n = snprintf(line, sizeof(line), "it is on line 60.. of...");
  write(fd, line,n);
  sleep(1);
  n = snprintf(line, sizeof(line), "nah just kidding\n\n");
  write(fd, line,n);

#if defined(CONFIG_BOARD_RESET_ON_CRASH)
  systemreset(false);
#endif
}
