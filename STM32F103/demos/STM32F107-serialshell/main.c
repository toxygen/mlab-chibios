/* J76 jumper must be in place */

#include <ch.h>
#include <hal.h>
#include "microshell2.h"

#include <chprintf.h>

/*===========================================================================*/
/* Generic code.                                                             */
/*===========================================================================*/

static WORKING_AREA(waShell, 2048);

/* shell thread */
static msg_t shell(void *arg)
{
	(void) arg;
	chRegSetThreadName("shell");
	start_shell();
}

/* leds heartbeat */
static WORKING_AREA(waThread1, 128);
static msg_t Thread1(void *arg) {
	
	(void)arg;
	chRegSetThreadName("blinker");
	while (TRUE) {
		palClearPad(GPIOB, GPIOB_LED1);
		chThdSleepMilliseconds(500);
		palSetPad(GPIOB, GPIOB_LED1);
		chThdSleepMilliseconds(500);
	}
}

int main(void) {
	Thread *shelltp = NULL;
	
	halInit();
	chSysInit();
	
	sdStart(&SD2, NULL);
	
	chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);
	
	while (TRUE) {
		if (!shelltp)
		{
			shelltp = chThdCreateStatic(waShell, sizeof(waShell),
										NORMALPRIO, shell, NULL);
		}
		else if (chThdTerminated(shelltp)) {
			shelltp = NULL;           /* Triggers spawning of a new shell.   */
		}
		chThdSleepMilliseconds(1000);
	}
}
