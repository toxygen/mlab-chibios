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

#include "fc_pid.h"
#include "fc_gen.h"

#include <math.h>

/*
 * Initialize the PID structure.
 * The K values are just 1, a value should not be used in real operation.
 */
void initPIDStruct(PIDStruct *pid)
{
	pid->K=1;
	pid->Kp=1;
	pid->Ki=0;
	pid->Kd=0;
	pid->lastError=0;
	pid->integratedError=0;
	pid->lastTicks=1;
}


/*
 * Get new PID results
 * The resolution of this PID controller is on tick.
 * @Arguments
 * pid							PID controller structure
 * targetPosition		Position we want to go to
 * currentPosition	Position we are currently at
 * ticks						Number of ticks (milliseconds)
 * @Returns
 * correction (-256 to 256)
 */
int16_t PIDUpdatePid(PIDStruct *pid, float currentPosition, int ticks)   {
	float error=0;			/* Current Error */
	uint16_t tickDiff=0;		/* Time difference in milliseconds */
	float pTerm=0.0;	/* Proportional term */
	float iTerm=0.0;	/* Integral term */
	float dTerm=0.0;	/* Differential term */
	int16_t outVal=0;	/* Output value to see at GDB */
		
	/*
	 * Calculate the shortest angle.
	 */
	error = GENCalcShortestAngle(pid->targetPos, currentPosition);

	/*
	 * PID code.
	 */
	tickDiff = ticks - pid->lastTicks; /* Time difference in milliseconds */

	pTerm = pid->Kp * error;
	pid->integratedError += error * tickDiff; 
	GENconstrain(pid->integratedError, -GUARD_GAIN, GUARD_GAIN);
	iTerm = pid->Ki * pid->integratedError;
	dTerm = pid->Kd * (error - pid->lastError) / tickDiff; /* Differential between last en current error, divided by time */

	/* 
	 * Next frame.
	 */
	pid->lastError = error;
	pid->lastTicks = ticks;

	outVal = -pid->K*(pTerm + iTerm + dTerm);
	GENconstrain(outVal, -OUT_GAIN, OUT_GAIN);
	return outVal;
}
