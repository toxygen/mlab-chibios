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

/*
 * Return smallest float value
 */
float minabsf(float val1, float val2);
#if 0
/*
 * Simple function to keep result of 2 variables between 'min' and 'max'
 */
int constrain(int value, int min, int max);

/*
 * Simple function to keep result of 2 variables between 'min' and 'max'
 */
int GENConstrainf(float value, float min, float max);
#endif

/*
 * Calculate the shortest angle.
 * For 2 angles in radiants 
 */
float GENCalcShortestAngle(float target, float pos);

/*
 * Calculate the mean of the mean of a float array
 */
float GENCalcVariance(float col[], size_t size);

/*
 * Variance test
 */
void GENTestVariance(void);

/*
 * Generic macro's
 */

/* 
 * constrain values to range.
 */
#define GENconstrain(v, vmin, vmax){																					\
	if (v <= vmin)																															\
		{ v = vmin; }																															\
	else if (v >= vmax)																													\
		{ v = vmax; }																															\
}

/*
 * When skipping cycles.
 */
#define nop() \
	   asm volatile ("nop")
