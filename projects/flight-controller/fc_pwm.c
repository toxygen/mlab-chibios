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

		Thanks http://www.x-firm.com/?page_id=193
*/

#include "ch.h"
#include "hal.h"

#include "fc_pwm.h"

/*
 * Setup for timer 3
 */
static PWMConfig pwmcfg3 = {
	1000000,                                    /* 1MHz PWM clock frequency.   */
	   3333,                                    /* Initial PWM period 1/200S.       */
	NULL,
	{
		{PWM_OUTPUT_ACTIVE_HIGH, NULL},
		{PWM_OUTPUT_ACTIVE_HIGH, NULL},
		{PWM_OUTPUT_ACTIVE_HIGH, NULL},
		{PWM_OUTPUT_ACTIVE_HIGH, NULL}
	},
	0,
#if STM32_PWM_USE_ADVANCED
	0
#endif
};

/*
 * Set pulse width on one of the 4 PWM channels.
 * The performance is set in microseconds.
 */
void PWMSetTimeHigh(uint8_t channel, uint16_t timeHigh)
{
	/*
	 * The time high needs to be multiplied by 3.
	 */
	pwmEnableChannel(&PWMD3, channel, PWM_PERCENTAGE_TO_WIDTH(&PWMD3, 3*timeHigh));
}

/*
 * Initialize the PWM output
 */
void PWMInit(void)
{
	/*
	 * Initializes the PWM driver 1 
	 */
	pwmStart(&PWMD3, &pwmcfg3);

	/*
	 * Remap the output of timer 3 PWM to PortC 6,7,8,9
	 */
	AFIO->MAPR |= 0x00000C00;		/* Remap Timer 3 to the remap ports */

	/*
	 * Timer 3 ports
	 */
	palSetPadMode(IOPORT3, 6, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
	palSetPadMode(IOPORT3, 7, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
	palSetPadMode(IOPORT3, 8, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
	palSetPadMode(IOPORT3, 9, PAL_MODE_STM32_ALTERNATE_PUSHPULL);

	/*
	 * Set all of the channels 100us, to show the PWM is not armed.
	 */
	PWMSetTimeHigh(0, 100);
	PWMSetTimeHigh(1, 100);
	PWMSetTimeHigh(2, 100);
	PWMSetTimeHigh(3, 100);
}

/*
 * Enable PWM channels
 */
void PWMEnable(void)
{
	/*
	 * Initializes the PWM driver 1 
	 */
	pwmStart(&PWMD3, &pwmcfg3);

	/*
	 * Remap the output of timer 3 PWM to PortC 6,7,8,9
	 */
	AFIO->MAPR |= 0x00000C00;		/* Remap Timer 3 to the remap ports */

	/*
	 * Set all of the channels 100us, to show the PWM is not armed.
	 */
	PWMSetTimeHigh(0, 100);
	PWMSetTimeHigh(1, 100);
	PWMSetTimeHigh(2, 100);
	PWMSetTimeHigh(3, 100);
}

/*
 * Disable PWM channels
 */
void PWMDisable(void)
{
	/*
	 * Initializes the PWM driver 1 
	 */
	pwmStop(&PWMD3);
}
