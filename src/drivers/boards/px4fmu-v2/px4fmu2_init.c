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

#include <arch/board/board.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_led.h>

#include <systemlib/px4_macros.h>
#include <systemlib/cpuload.h>
#include <systemlib/perf_counter.h>

#include <systemlib/hardfault_log.h>

#if defined(CONFIG_HAVE_CXX) && defined(CONFIG_HAVE_CXXINITIALIZE)
#include <systemlib/systemlib.h>
#endif

/* todo: This is constant but not proper */
__BEGIN_DECLS
extern void led_off(int led);
__END_DECLS

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

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

	/* Using Battery Backed Up SRAM */

	int filesizes[CONFIG_STM32_BBSRAM_FILES+1] = BSRAM_FILE_SIZES;
	int nfc = stm32_bbsraminitialize(BBSRAM_PATH, filesizes);

        syslog(LOG_INFO, "[boot] %d Battery Backed Up File(s) \n",nfc);

#if defined(CONFIG_STM32_SAVE_CRASHDUMP)

        /* Panic Logging in Battery Backed Up Files */
        int hadCrash = check_hardfault_satus("boot");
        if (hadCrash == OK) {
            write_hardfault("boot", fileno(stdout), HARDFAULT_DISPLAY_FORMAT, false);
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

//todo:need to be on the stack or on ans unallocated label.
// This is bacause (#pragma ignored not working for -Werror=frame-larger-than=]
fullcontext_s dump;

__EXPORT void board_crashdump(uint32_t currentsp, void *tcb, uint8_t *filename, int lineno)
{
//  fullcontext_s dump;

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
      memcpy(&dump.context.proc.xcp.regs, (void*)current_regs, sizeof(dump.context.proc.xcp.regs));
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


#if defined(CONFIG_BOARD_RESET_ON_CRASH)
  systemreset(false);
#endif
}
