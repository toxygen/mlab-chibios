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

#include <math.h>

#include "fc_uart.h"
#include "fc_gen.h"

/*
 * Return smallest absolute float value
 */
float minabsf(float val1, float val2)
{
	if(fabs(val1) < fabs(val2)) {
		return val1;
	} else {
		return val2;
	}
}

/*
 * Calculate the shortest angle.
 * For 2 angles in radiants 
 */
float GENCalcShortestAngle(float target, float pos)
{
	float error=0.0;

	error = target - pos;
	if (error > M_PI) {
		error -= 2*M_PI;
	}
	else if (error < -M_PI){
		error += 2*M_PI;
	}

	return error;
}

/*
 * Calculate the mean of the mean of a float array
 */
float GENCalcVariance(float col[], size_t size)
{
	size_t	count=0;
	float   total=0.0;
	float   mean=0.0;
	float   meanSquare=0.0;
	float		variance=0.0;

	/*
	 * Calculate mean square
	 */
	for(count=0; count < size; count++){
		total+=col[count]*col[count];
	}
	meanSquare=total/size;
	UARTPrintf("Mean Square:	%f\r\n",meanSquare);

	/*
	 * Calculate mean
	 */
	total=0.0;
	for(count=0; count < size; count++){
		total+=col[count];
	}
	mean=total/size;
	UARTPrintf("Mean:	%f\r\n", mean);

	/*
   * Calculate variance
	 */
	variance=meanSquare-(size/(size-1))*mean*mean;
	UARTPrintf("Variance:	%f\r\n", variance);

	/*
	 * Done!
	 */
	return variance;
}

/*
 * Variance test
 */
float numbers[8] = { 2.0, 4.0, 4.0, 4.0, 5.0, 5.0, 7.0, 9.0};
void GENTestVariance(void)
{
	float variance=0.0;
	
	UARTPrintf("Enter Variance\r\n");
	variance = GENCalcVariance(numbers, 8);
	UARTPrintf("Variance:	%f\r\n", variance);
	chThdSleepMilliseconds(10000);
}
