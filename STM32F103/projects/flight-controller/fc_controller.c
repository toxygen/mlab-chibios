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
#include <string.h>
#include <math.h>

#include "fc_controller.h"
#include "fc_pid.h"
#include "fc_mpu6050.h"
#include "fc_pwm.h"
#include "fc_gen.h"

/*
 * Structures holding the X, Y and Z axis controllers
 */
PIDStruct controlX;
PIDStruct controlY;
PIDStruct controlZ;

/*
 * Mutex to lock output buffer
 */
static Mutex					controlMtx; /* Mutex */

/*
 * Structure to store results in.
 */
ControlStruct controlStruct;
PWMoutput			PWMout;

/*
 * Initialize controller.
 */
void CONTROLInit(void)
{
	/* 
	 * Setup the PID controller structures.
	 */
	initPIDStruct(&controlX);
	initPIDStruct(&controlY);
	initPIDStruct(&controlZ);

	/*
	 * Initialize Mutex
	 */
	chMtxInit(&controlMtx); /* Mutex initialization before use */

	/*
	 * Clear controlStruct.
	 */
	memset(&controlStruct, 0, sizeof(ControlStruct));

	/*
	 * DEBUG set aiming values hard for now
	 */
	controlX.Kp=100.0;
	controlY.Kp=100.0;
	controlZ.Kp=10.0;
	controlX.targetPos = M_PI/2;
	controlY.targetPos = M_PI/2;

	/*
	 * Set default power to the output.
	 */
	controlStruct.power=3000;

}

/*
 * Get results out.
 */
void CONTROLGetOut(ControlStruct *out)
{
	chMtxLock(&controlMtx);
	memcpy(out, &controlStruct, sizeof(ControlStruct));
	chMtxUnlock();
}

/*
 * Routine to put the structure hold content to the PWM out.
 */
static void CONTROLPWMOut(void)
{
	/*
	 * Put the 4 values to the PWM out.
	 */
	chMtxLock(&controlMtx);
	PWMSetTimeHigh(0, PWMout.m1);
	PWMSetTimeHigh(1, PWMout.m2);
	PWMSetTimeHigh(2, PWMout.m3);
	PWMSetTimeHigh(3, PWMout.m4);
	chMtxUnlock();
}

/*
 * Calculate the Power distribution over the 4 motors.
 * Provided with controller structure the PWMoutput structure gets filled with the appropriate 
 * The algorithm is configured the fly the Quad copter in X mode.
 * With motor configuration :
 *
 * 1   2
 *   X
 * 4   3
 *
 * 1, 3 rotate clockwise
 * 2, 4 rotate counter clockwise
 */
static void CONTROLCalcPower(void)
{
	uint16_t qpower; /* Holds power/4 */
	/*
	 * Distribute the Throttle over the 4 Motors
	 */
	chMtxLock(&controlMtx);
	qpower  = controlStruct.power/4;
	PWMout.m1 = qpower;
	PWMout.m2 = qpower;
	PWMout.m3 = qpower;
	PWMout.m4 = qpower;

	/*
	 * Calculate Z axis correction
	 */
	PWMout.m1 += controlStruct.z;
	PWMout.m3 += controlStruct.z;
	PWMout.m2 -= controlStruct.z;
	PWMout.m4 -= controlStruct.z;

	/*
	 * Calculate Y axis correction
	 */
	PWMout.m1 += controlStruct.y;
	PWMout.m4 += controlStruct.y;
	PWMout.m2 -= controlStruct.y;
	PWMout.m3 -= controlStruct.y;

	/*
	 * Calculate X axis correction
	 */
	PWMout.m1 += controlStruct.x;
	PWMout.m2 += controlStruct.x;
	PWMout.m3 -= controlStruct.x;
	PWMout.m4 -= controlStruct.x;
	chMtxUnlock();
}

/*
 * To make sure no motor cuts in mid air.
 * Output should me held between 500-2000 us.
 */
static void CONTROLLimitOut(void)
{
	chMtxLock(&controlMtx);

	GENconstrain(PWMout.m1, PWM_MIN_HIGH, PWM_MAX_HIGH);
	GENconstrain(PWMout.m2, PWM_MIN_HIGH, PWM_MAX_HIGH);
	GENconstrain(PWMout.m3, PWM_MIN_HIGH, PWM_MAX_HIGH);
	GENconstrain(PWMout.m4, PWM_MIN_HIGH, PWM_MAX_HIGH);

	chMtxUnlock();
}

/*
 * Get current motor pwm output state.
 */
void CONTROLGetMotorOut(PWMoutput *out)
{
	chMtxLock(&controlMtx);
	memcpy(out, &PWMout, sizeof(PWMoutput));	
	chMtxUnlock();
}

/*
 * Loop to update the PID output towards the PWM module.
 */
void CONTROLUpdate(void)
{
	MPU6050OutputData gyroData;		/* Structure holding the orientation data. */
	
	int timeNow=0;		/* Time now in ms */

	/*
	 * Get input values for the 3 axis.
	 */
	MPU6050GetData(&gyroData);

	/*
	 * Get the current time in ms.
	 */
	timeNow = MS2ST(chTimeNow());

	/*
	 * Update the output values
	 */
	chMtxLock(&controlMtx);
	controlStruct.x = PIDUpdatePid(&controlX, gyroData.x_angle, timeNow);
	controlStruct.y = PIDUpdatePid(&controlY, gyroData.y_angle, timeNow);
	controlStruct.z = PIDUpdatePid(&controlZ, gyroData.z_angle, timeNow);
	chMtxUnlock();

	/*
	 * Put the output values towards power distribution calculations.
	 */
	CONTROLCalcPower();

	/*
	 * Limit the controller output to sane values.
	 */
	CONTROLLimitOut();

	/*
	 * Send the results to the PWM out channels.
	 */
	CONTROLPWMOut();
}

