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
 * This is a simple kalman filter implementation.
 * This filter asumes the current measure ment close to the last one is the correct one.
 * This function can be used to even out slow changing precesses, like bearing and altitude.
 * sample values for float Q = 0.022; R = 0.617;
 */
float KALsimple(float z_measured, float x_est_last, float Q, float R);

typedef struct {
	float Q_angle;
	float Q_gyro;
	float R_angle;

	float x_angle;
	float x_bias;
	float P_00; 
	float	P_01; 
	float P_10; 
	float P_11;
} kalmanStruct;

/*
 * Initialize the Kalman structure.
 */
void KALInit(kalmanStruct *kalStruct);

/*
 * Insert Accelerometer Angle, and Gyroscope rate + time elapsed to get none-drifting angle data.
 * All input should be in Radiants.
 * The Kalman structure handles the variables that are stored from one iteration to the other.
 */
float KALCalculate(kalmanStruct *kalStruct, float newAngle, float newRate,int looptime);

