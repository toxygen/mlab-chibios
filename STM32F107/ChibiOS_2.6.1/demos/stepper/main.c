/*
 ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio
 
 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at
 
 http://www.apache.org/licenses/LICENSE-2.0
 
 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
 */

#include <stdio.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "test.h"

#include "shell.h"
#include "chprintf.h"

#include "microusb.h"
#include "microspi.h"
#include "dspin.h"



/*===========================================================================*/
/* Command line related.                                                     */
/*===========================================================================*/

#define SHELL_WA_SIZE   THD_WA_SIZE(2048)
#define TEST_WA_SIZE    THD_WA_SIZE(256)

static void cmd_mem(BaseSequentialStream *chp, int argc, char *argv[]) {
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

static void cmd_threads(BaseSequentialStream *chp, int argc, char *argv[]) {
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

static void cmd_test(BaseSequentialStream *chp, int argc, char *argv[]) {
	Thread *tp;
	
	(void)argv;
	if (argc > 0) {
		chprintf(chp, "Usage: test\r\n");
		return;
	}
	tp = chThdCreateFromHeap(NULL, TEST_WA_SIZE, chThdGetPriority(),
							 TestThread, chp);
	if (tp == NULL) {
		chprintf(chp, "out of memory\r\n");
		return;
	}
	chThdWait(tp);
}

static void cmd_write(BaseSequentialStream *chp, int argc, char *argv[]) {
	static uint8_t buf[] =
	"0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
	"0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
	"0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
	"0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
	"0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
	"0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
	"0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
	"0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
	"0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
	"0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
	"0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
	"0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
	"0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
	"0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
	"0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
	"0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";
	
	(void)argv;
	if (argc > 0) {
		chprintf(chp, "Usage: write\r\n");
		return;
	}
	
	while (chnGetTimeout((BaseChannel *)chp, TIME_IMMEDIATE) == Q_TIMEOUT) {
		chSequentialStreamWrite(&SDU1, buf, sizeof buf - 1);
	}
	chprintf(chp, "\r\n\nstopped\r\n");
}

static const ShellCommand commands[] = {
	{"mem", cmd_mem},
	{"threads", cmd_threads},
	{"test", cmd_test},
	{"write", cmd_write},
	{NULL, NULL}
};

static const ShellConfig shell_cfg1 = {
	(BaseSequentialStream *)&SDU1,
	commands
};

/*===========================================================================*/
/* Generic code.                                                             */
/*===========================================================================*/

/*
 * Red LED blinker thread, times are in milliseconds.
 */
static WORKING_AREA(waThread1, 128);
static msg_t Thread1(void *arg) {
	
	(void)arg;
	chRegSetThreadName("blinker");
	while (TRUE) {
		palSetPad(GPIOB, GPIOB_LED_STATUS1);
		chThdSleepMilliseconds(100);
		palClearPad(GPIOB, GPIOB_LED_STATUS1);
		chThdSleepMilliseconds(100);
		palSetPad(GPIOB, GPIOB_LED_STATUS1);
		chThdSleepMilliseconds(100);
		palClearPad(GPIOB, GPIOB_LED_STATUS1);
		chThdSleepMilliseconds(700);
	}
	return 0;
}
/*
 * Application entry point.
 */
int main(void) {
	Thread *shelltp = NULL;
	
	/*
	 * System initializations.
	 * - HAL initialization, this also initializes the configured device drivers
	 *   and performs the board-specific initializations.
	 * - Kernel initialization, the main() function becomes a thread and the
	 *   RTOS is active.
	 */
	halInit();
	chSysInit();
	
	initSPI();
	palSetPadMode(GPIOC, 4, PAL_MODE_INPUT);     /* MISO.*/
	initUsb();
	/*
	 * Shell manager initialization.
	 */
	shellInit();
	
	/*
	 * Creates the blinker thread.
	 */
	chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);
	dSPIN_Run(REV, Speed_Steps_to_Par(50));
	/*
	 * Normal main() thread activity, in this demo it does nothing except
	 * sleeping in a loop and check the button state.
	 */
	while (TRUE) {
		if (!shelltp && (SDU1.config->usbp->state == USB_ACTIVE))
			shelltp = shellCreate(&shell_cfg1, SHELL_WA_SIZE, NORMALPRIO);
		else if (chThdTerminated(shelltp)) {
			chThdRelease(shelltp);    /* Recovers memory of the previous shell.   */
			shelltp = NULL;           /* Triggers spawning of a new shell.        */
		}
		chThdSleepMilliseconds(1000);
	}
}
