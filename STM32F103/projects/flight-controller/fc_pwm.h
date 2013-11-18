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

/*
 * Define the minimal and maximal valid PWM values.
 * In micro seconds.
 */
#define PWM_MAX_HIGH 2000
#define PWM_MIN_HIGH 500

/*
 * Initialize the PWM output
 */
void PWMInit(void);

/*
 * Enable PWM channels
 */
void PWMEnable(void);

/*
 * Disable PWM channels
 */
void PWMEnable(void);


/*
 * Set pulse width on one of the 4 PWM channels.
 * The performance is set in microseconds.
 */
void PWMSetTimeHigh(uint8_t channel, uint16_t timeHigh);

