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
 * Union holding compass data.
 */
typedef union 
{
	struct
	{
		uint8_t x_compass_h;
		uint8_t x_compass_l;
		uint8_t z_compass_h;
		uint8_t z_compass_l;
		uint8_t y_compass_h;
		uint8_t y_compass_l;
	} reg;
	struct
	{
		int16_t x_compass;
		int16_t z_compass;
		int16_t y_compass;
	} value;
} compass_union;

/*
 * Initialize the BMP085 driver.
 */
uint8_t HMC5883Init(void);

/*
 * Get Compass data
 */
void HMC5883GetData(compass_union *compassPtr);

/*
 * Get heading in radiants.
 */
float HMC5883GetHeading(void);

/*
 * Update sensor data.
 * This method should only be called by the update thead.
 */
void HMC5883Update(void);
