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

#include "ch.h"
#include "hal.h"

/*
 * Turn LED on
 * Arguments
 * num	GPIOB_LED1, GPIOB_LED2
 */
void ledon1(void *num) {
	(void)num;
  palClearPad(IOPORT2, GPIOB_LED1);
}

/*
 * Turn LED off
 * Arguments
 * num	GPIOB_LED1, GPIOB_LED2
 */
void ledoff1(void *num) {
	(void)num;
  palSetPad(IOPORT2, GPIOB_LED1);
}

/*
 * Toggle LED 2
 */
void ledtoggle1(void){
	palTogglePad(IOPORT1, GPIOB_LED2);
}

/*
 * Turn LED on
 * Arguments
 * num	GPIOB_LED1, GPIOB_LED2
 */
void ledon2(void *num) {
	(void)num;
  palClearPad(IOPORT2, GPIOB_LED2);
}

/*
 * Turn LED off
 * Arguments
 * num	GPIOB_LED1, GPIOB_LED2
 */
void ledoff2(void *num) {
	(void)num;
  palSetPad(IOPORT2, GPIOB_LED2);
}

/*
 * Toggle LED 2
 */
void ledtoggle2(void){
	palTogglePad(IOPORT2, GPIOB_LED2);
}

/*
 * Endless Blink
 * ms blink speed in ms
 */
void endlessblink(int ms)
{
	while(1==1) {
		ledon2(0);
		ledon1(0);
		chThdSleepMilliseconds(ms);
		ledoff2(0);
		ledoff1(0);
		chThdSleepMilliseconds(ms);
	}
}
