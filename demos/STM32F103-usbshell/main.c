/* J76 jumper must be in place */

#include <ch.h>
#include <chprintf.h>
#include <string.h>

#include "microusb.h"
#include "microshell2.h"

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
	return (msg_t) 0; /* never reached */
}

/* leds heartbeat */
static WORKING_AREA(waThread1, 128);

static void Thread1(void *arg) {
	
	(void)arg;
	chRegSetThreadName("blinker");
	while (!chThdShouldTerminate()) {
		palClearPad(GPIOB, GPIOB_LED1);
		chThdSleepMilliseconds(500);
		palSetPad(GPIOB, GPIOB_LED1);
		chThdSleepMilliseconds(500);
	}
	chThdExit(1);
}

/*===========================================================================*/
/* User shell commands.                                                      */
/*===========================================================================*/

void cmd_start_led(int argc, char * argv[])
{
	(void) argc;
	(void) argv;
	
	chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, (tfunc_t)Thread1, NULL);
	chprint("led blinking thread started\r\n");
}

//char *test2_keys[] = {"bu", "cu", "du"};
void cmd_stop_led(int argc, char * argv[])
{
	(void) argc;
	(void) argv;

	Thread * tp = chRegFirstThread();
	if(tp)
	{
		while (strcmp(tp->p_name, "blinker") != 0) {
			tp = chRegNextThread(tp);
		}
	}
	chThdTerminate(tp);
	chThdWait(tp);
	chprint("led blinking thread killed\r\n");
}

ShellCommand user_commands[] = {

	{"stop_led",  cmd_stop_led},
	{"start_led", cmd_start_led},
	{NULL, NULL}
};


int main(void) {
	Thread *shelltp = NULL;
	
	halInit();
	chSysInit();
	
	init_usb();
	
	chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, (tfunc_t)Thread1, NULL);

	while (TRUE) {
		if (!shelltp && (SDU1.config->usbp->state == USB_ACTIVE))
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
