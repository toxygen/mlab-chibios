/*
 ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
 2011,2012 Giovanni Di Sirio.
 
 This file is part of ChibiOS/RT.
 
 ChibiOS/RT is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 3 of the License, or
 (at your option) any later version.
 
 ChibiOS/RT is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 
 ---
 
 A special exception to the GPL can be applied should you wish to distribute
 a combined work that includes ChibiOS/RT, without being obliged to provide
 the source code for any proprietary components. See the file exception.txt
 for full details of how and when the exception can be applied.
 */
#include <stdlib.h>
#include <string.h>

#include <ch.h>
#include <hal.h>
#include <test.h>
#include <shell.h>
#include <evtimer.h>
#include <chprintf.h>
#include "i2c_pns.h"

#include "ff.h"

int connected = 0;
int tried = 0;

/*===========================================================================*/
/* Command line related.                                                     */
/*===========================================================================*/

#define SHELL_WA_SIZE   THD_WA_SIZE(2048)
#define TEST_WA_SIZE    THD_WA_SIZE(256)

static void cmd_mem(BaseChannel *chp, int argc, char *argv[])
{
	size_t n, size;
	
	(void)argv;
	if (argc > 0) {
		chprintf(chp, "Usage: mem\r\n");
		return;
	}
	n = chHeapStatus(NULL, &size);
	chprintf(chp, "core free memory : %u bytes\r\n", chCoreStatus());
	chprintf(chp, "heap fragments   : %u\r\n", n);
	chprintf(chp, "heap free total  : %u bytes\r\n", size);
}

static void cmd_threads(BaseChannel *chp, int argc, char *argv[])
{
	static const char *states[] = {THD_STATE_NAMES};
	Thread *tp;
	
	(void)argv;
	if (argc > 0) {
		chprintf(chp, "Usage: threads\r\n");
		return;
	}
	chprintf(chp, "    addr    stack prio refs     state time\r\n");
	tp = chRegFirstThread();
	do {
		chprintf(chp, "%.8lx %.8lx %4lu %4lu %9s %lu\r\n",
				 (uint32_t)tp, (uint32_t)tp->p_ctx.r13,
				 (uint32_t)tp->p_prio, (uint32_t)(tp->p_refs - 1),
				 states[tp->p_state], (uint32_t)tp->p_time);
		tp = chRegNextThread(tp);
	} while (tp != NULL);
}

static void cmd_test(BaseChannel *chp, int argc, char *argv[])
{
	Thread *tp;
	
	(void)argv;
	if (argc > 0) {
		chprintf(chp, "Usage: test\r\n");
		return;
	}
	tp = chThdCreateFromHeap(NULL, TEST_WA_SIZE, chThdGetPriority(),
							 TestThread, chp);
	chprintf(chp, "thread created\r\n");
	if (tp == NULL) {
		chprintf(chp, "out of memory\r\n");
		return;
	}
	chThdWait(tp);
}

bool_t mag_initialized = 0;

static void cmd_mag(BaseChannel *chp, int argc, char *argv[])
{
	/* supress compiler warnings */
	(void)argc;
	(void)argv;
	
	HMC5883L_result result;
	if (!mag_initialized) {
		I2CInit_pns();
		mag_initialized = 1;
	}
	while (1) {
		chThdSleepMilliseconds(150);
		read_data_HMC5883L(&result);
		chprintf(chp, "X: %d\tY: %d\tZ: %d\tvalid: %d\r\n",
				 result.x, result.y, result.z, result.valid);
	}
}

static const ShellCommand commands[] = {
	{"mem", cmd_mem},
	{"threads", cmd_threads},
	{"test", cmd_test},
	{"mag", cmd_mag},
	{NULL, NULL}
};

static const ShellConfig shell_cfg1 = {
	(BaseChannel *)&SD2,
	commands
};

/*===========================================================================*/
/* Main and generic code.                                                    */
/*===========================================================================*/

/* Working area for the LED flashing thread. */
static WORKING_AREA(staticWA, 1280);

/* LED flashing thread. */
static msg_t staticThread(void *arg)
{
	(void)arg;
	while(1)
	{
		palTogglePad(GPIOB, GPIOB_LED1);
		chThdSleepMilliseconds(200);
	}
	return 0;
}

/*
 * main ()
 */

int main(void)
{
	Thread *shelltp = NULL;

	halInit();
	chSysInit();

	sdStart(&SD2, NULL);

	/* Start the shell */
	shellInit();
	
	(void)chThdCreateStatic(staticWA, sizeof(staticWA), HIGHPRIO,
							staticThread, NULL);
	while(1)
	{
		if (!shelltp)
		{
			shelltp = shellCreate(&shell_cfg1, SHELL_WA_SIZE, NORMALPRIO);
		}
		else if (chThdTerminated(shelltp))
		{
			chThdRelease(shelltp); /* Recovers memory of the previous shell. */
			shelltp = NULL;        /* Triggers spawning of a new shell.      */
		}
	}
}