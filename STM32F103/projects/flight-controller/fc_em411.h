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
 * Structure representation of the GPS fix data.
 * $GPGGA,161229.487,3723.2475,N,12158.3416,W,1,07,1.0,9.0,M,,,,0000*18
 *
 */
typedef enum { 
	fixNa,				// Fix not available
	gpsSps,				// GPS SPS Mode, fix valid
	difGpsSps,		// Differential GPS, SPS Mode , fix valid
	gpsPps				// GPS PPS Mode, fix valid
} posFixEnum;

/*
 * Structure holding the GGA data.
 */
typedef struct {
	uint16_t		hour;			// hours
	uint16_t		min;			// minutes
	uint16_t		sec;			// seconds
	uint16_t		ms;				// milliseconds
	float				decLat;		// Decimal latitude
	float				decLong;	// Decimal longitude
	posFixEnum	posFix;		// Position fix indicator
	uint16_t		satsUsed; // Satellites used to get fix.
	float				hdop;			// Horizontal Dilution of Precision
	float				alt;			// Altitude in meters
} EM411FixData;

/*
 * structure holding latitude and longitude information.
 */
typedef struct {
	float				decLat;
	float				decLong;
} EM411Coords;

/*
 * Structure holding the VTG data.
 */
typedef struct {
	float course;		/* Course in degrees, composed from GPS */ 
	float	speed;		/* Speed in m/s */
} EM411CourseSpeed;


/*
 * Initialize the EM-411 GPS receiver.
 */
void EM411Init(void);

/*
 * Read and update the current GPS data.
 */
void EM411Update(void);

/*
 * Get the last data emea message.
 */
void EM411GetLast(char *lastOut);

/*
 * Get gpgga data (GPS fix data)
 */
void EM411GetGpgga(EM411FixData *fixOut);

/*
 * Get gpvtg data (Course and speed data)
 */
void EM411GetGpvtg(EM411CourseSpeed *csOut);

/*
 * Place coordinates from fix into coordinates structure.
 */
void EM411FixToCoord(EM411FixData *in, EM411Coords *out);

/*
 * Calculate the distance between to GPS coordinates.
 * We assume a flat earth and use Pythagoras to calculate the distance between 2 points.
 */
float EM411GetDistance(EM411Coords *coord1, EM411Coords *coord2);

/*
 * Calculate the course that should be taken to go from coord1 -> coord2
 * Course is radians.
 */
float EM411GetCourse(EM411Coords *coord1, EM411Coords *coord2);


#if 0
/*
 * Just for testing
 */
void  EM411TestTan(void);
#endif
