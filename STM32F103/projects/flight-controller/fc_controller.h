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
 * Structure to hold output values in us (microseconds)
 * The 'ON' period of the signal should remain between 1000 and 2000 us.
 * The PWM signal period will be 1/200 second.
 */
typedef struct {
	uint16_t m1; /* Motor 1 front left */
	uint16_t m2; /* Motor 2 front right */
	uint16_t m3; /* Motor 3 back left */
	uint16_t m4; /* Motor 4 back right */
} PWMoutput;

/*
 * Controller input values.
 */
typedef struct {
	int power;	/* Input power (the amount of thrust */
	int x;			/* X correction factor */
	int y;			/* Y correction factor */
	int z;			/* Z correction factor */
} ControlStruct;

/*
 * Initialize controller.
 */
void CONTROLInit(void);

/*
 * Loop to update the PID output towards the PWM module.
 */
void CONTROLUpdate(void);

/*
 * Get results PID out.
 */
void CONTROLGetOut(ControlStruct *out);

/*
 * Get current motor pwm output state.
 */
void CONTROLGetMotorOut(PWMoutput *out);
