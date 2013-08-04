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
 * Structure holding calibration data.
 */
typedef struct {
	short						ac1;
	short						ac2;
	short						ac3;
	unsigned short	ac4;
	unsigned short	ac5;
	unsigned short  ac6;
	short						b1;
	short						b2;
	short						mb;
	short						mc;
	short						md;
} BMP085CalData;

typedef struct {
	long		x1;
	long		x2;
	long		b5;
	long		temp;
	uint8_t oss;
} BMP085Temp;

/*
 * Get the calibration data.
 */
msg_t BMP085GetCalData(void);

/*
 * Get the pressure
 */
long BMP085GetPressure(void);

/*
 * Get altitude relative to Altitude at Initialization
 */
float BMP085GetAltitude(void);

/*
 * Get the altitude compensated using a Kalman filter.
 */
float BMP085GetKalmanAltitude(void);

/*
 * Initialize the BMP085 driver.
 */
void BMP085Init(void);

/*
 * Update pressure in register
 * This method should only be executed by the thread.
 */
void BMP085UpdatePressure(void);

/*
 * Get the current temperature
 */
short BMP085GetTemp(void);

/*
 * Take 20 samples, use them to calculate the variance in the measurements.
 */
float BMP085GetVariance(void);

