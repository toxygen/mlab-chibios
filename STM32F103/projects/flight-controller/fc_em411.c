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
#include "chprintf.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "fc_em411.h"
#include "fc_uart.h"

#define EMEA_BUFFER_SIZE 141

/*
 * Mutex to lock output buffer
 */
static Mutex					em411mtx; /* Mutex */

/*
 * Buffer to store the most recent EMEA message.
 */
static char lastMsg[EMEA_BUFFER_SIZE];
static char curMsg[EMEA_BUFFER_SIZE];

/*
 * Structure to store GPS fix data.
 */
static EM411FixData			fixData;
static EM411CourseSpeed courseSpeed;

/*
 * This Function takes the NMEA command registered.
 * The command should to sent complete with this function adding the checksum in
 * Example "$PSRF103,05,00,00,01*00\r\n" Becomes "$PSRF103,05,00,00,01*21\r\n"
 */
static void EM411NMEAChecksum(char msg[])
{
	uint8_t cur=1; // 1 Because we want to skip the leading '$'
	uint8_t sum=0;

	/*
	 * Calculate checksum
	 */
	while(msg[cur] != '*' && cur < strlen(msg) ){
		sum ^= msg[cur];
		cur++;
	}
	cur++; /* Step over the '*' */

	/*
	 * Put the checksum in place
	 */
	sprintf((msg+cur), "%2x\r\n", sum);
}

static void EM411Commands(void)
{
	char nmeaMsg[140]; /* Store NMEA messages */

	/*
	 * Disable unneeded messages from gps module.
	 *  0=GGA,1=GLL,2=GSA,3=GSV,4=RMC,5=VTG
	 */

	/*
	 * Enable GGA
	 */
	strcpy(nmeaMsg, "$PSRF103,00,00,01,01*00\r\n");
	EM411NMEAChecksum(nmeaMsg);
	sdWrite(&SD2, (uint8_t*) nmeaMsg, strlen(nmeaMsg));

	/*
	 * Disable GLL
	 */
	strcpy(nmeaMsg, "$PSRF103,01,00,00,01*00\r\n");
	EM411NMEAChecksum(nmeaMsg);
	sdWrite(&SD2, (uint8_t*) nmeaMsg, strlen(nmeaMsg));

	/*
	 * Disable GSA
	 */
	strcpy(nmeaMsg, "$PSRF103,02,00,00,01*00\r\n");
	EM411NMEAChecksum(nmeaMsg);
	sdWrite(&SD2, (uint8_t*) nmeaMsg, strlen(nmeaMsg));

	/*
	 * Disable GSV
	 */
	strcpy(nmeaMsg, "$PSRF103,03,00,00,01*00\r\n");
	EM411NMEAChecksum(nmeaMsg);
	sdWrite(&SD2, (uint8_t*) nmeaMsg, strlen(nmeaMsg));

	/*
	 * Disable RMC
	 */
	strcpy(nmeaMsg, "$PSRF103,04,00,00,01*00\r\n");
	EM411NMEAChecksum(nmeaMsg);
	sdWrite(&SD2, (uint8_t*) nmeaMsg, strlen(nmeaMsg));

	/*
	 * Enable VTG
	 */
	strcpy(nmeaMsg, "$PSRF103,05,00,01,01*00\r\n");
	EM411NMEAChecksum(nmeaMsg);
	sdWrite(&SD2, (uint8_t*) nmeaMsg, strlen(nmeaMsg));
}

/*
 * Initialize the EM-411 GPS receiver.
 */
void EM411Init(void)
{

	/*
	 * Clear buffers.
	 */
	memset(lastMsg, 0, EMEA_BUFFER_SIZE);
	memset(curMsg, 0, EMEA_BUFFER_SIZE);

	/*
	 * Initialize Mutex
	 */
	chMtxInit(&em411mtx); /* Mutex initialization before use */

	/*
	 * Send commands to initialize GPS module.
	 */
	EM411Commands();
}

void EM411FillGpgga(char *gpsMsg)
{
	uint16_t		latdeg=0;
	uint16_t		latmin=0;
	uint16_t		latminmin=0;
	char				ns=' ';
	uint16_t		longdeg=0;
	uint16_t		longmin=0;
	uint16_t		longminmin=0;
	char				ew=' ';

	/*
	 * Only capturing ints to keep stack usage down.
	 */
	sscanf(gpsMsg,
			"$GPGGA,%2hd%2hd%2hd.%hd,%2hd%2hd.%4hd,%c,%3hd%2hd.%4hd,%c,%d,%hd",
			&(fixData.hour),
			&(fixData.min),
			&(fixData.sec),
			&(fixData.ms),
			&latdeg,
			&latmin,
			&latminmin,
			&ns,
			&longdeg,
			&longmin,
			&longminmin,
			&ew,
			(int*) &(fixData.posFix),
			&(fixData.satsUsed)
			);

	/*
	 * Do calculations
	 */
	fixData.decLat = ((latmin*10000+latminmin)/600000.0)+latdeg;
	fixData.decLong = ((longmin*10000+longminmin)/600000.0)+longdeg;

	if(ns == 'S') {
		fixData.decLat *= -1;
	}
	if(ew == 'W') {
		fixData.decLong *= -1;
	}
}

/*
 * Fill the structure holding the VTG message.
 * $GPVTG,309.62,T,,M,0.13,N,0.2,K*6E
 */
static void EM411FillGpvtg(char *gpsMsg)
{
	uint16_t	deg=0;
	uint16_t	degFrac=0;
	uint16_t	knotSpeed=0;
	uint16_t	knotSpeedFrac=0;
										
	/*
	 * Reason for not using floats in sscanf is the big stack it requires.
	 */
	sscanf(gpsMsg,
			"$GPVTG,%hd.%hd,T,,M,%hd.%hd,N",
			&deg,
			&degFrac,
			&knotSpeed,
			&knotSpeedFrac);

	/* 
	 * Build the output values
	 */
	courseSpeed.course=deg + degFrac * 0.01; /* Degrees in float */
	courseSpeed.speed=(knotSpeed + knotSpeedFrac*0.01)/0.514; /* Speed in m/s */
}

/*
 * Dispatch the different GPS messages to the decoders
 */
static void EM411Decode(char *gpsMsg)
{
	/*
	 * GGA-Global Positioning System Fixed Data
	 */
	if(strncmp(gpsMsg, "$GPGGA", 6) == 0) {
		EM411FillGpgga(gpsMsg);
	}
	else if(strncmp(gpsMsg, "$GPVTG", 6) == 0) {
		EM411FillGpvtg(gpsMsg);
	}
}

/*
 * Read and update the current GPS data.
 */
void EM411Update(void)
{
	int  count=0;

	/*
	 * Get message
	 */
	while((curMsg[count] = chIOGet(&SD2)) != '\n' && count < EMEA_BUFFER_SIZE) {
		//curMsg[count] = chIOGet(&SD2);
		count++;
	}

	/*
	 * Copy message to output buffer
	 * Also fill data structure.
	 */
	chMtxLock(&em411mtx);
	strncpy(lastMsg, curMsg, EMEA_BUFFER_SIZE-1);
	EM411Decode(lastMsg);
	chMtxUnlock();

	/*
	 * Clear buffer.
	 */
	memset(curMsg, 0, EMEA_BUFFER_SIZE);
}

/*
 * Get the last data emea message.
 */
void EM411GetLast(char *lastOut)
{
	chMtxLock(&em411mtx);
	strncpy(lastOut, lastMsg, EMEA_BUFFER_SIZE-1);
	chMtxUnlock();
}

/*
 * Get gpgga data (GPS fix data)
 */
void EM411GetGpgga(EM411FixData *fixOut)
{
	chMtxLock(&em411mtx);
	memcpy(fixOut, &fixData, sizeof(EM411FixData));	
	chMtxUnlock();
}

/*
 * Get gpvtg data (Course and speed data)
 */
void EM411GetGpvtg(EM411CourseSpeed *csOut)
{
	chMtxLock(&em411mtx);
	memcpy(csOut, &courseSpeed, sizeof(EM411CourseSpeed));	
	chMtxUnlock();
}

/*
 * Place coordinates from fix into coordinates structure.
 */
void EM411FixToCoord(EM411FixData *in, EM411Coords *out)
{
	out->decLat		= in->decLat;
	out->decLong	= in->decLong;
}

/*
 * Calculate the distance between to GPS coordinates.
 * We assume a flat earth and use Pythagoras to calculate the distance between 2 points.
 */
float EM411GetDistance(EM411Coords *coord1, EM411Coords *coord2)
{
	float degreeHight = 111319.4; /* a 360th part of the circumvention of the earth. */
	float degreeWidth = 0.0;
	float dist = 0.0; /* Distance in meters. */
	float diffLat = 0.0;
	float diffLong = 0.0;

	/*
	 * Calculate the width of a degree at a given latitude.
	 * We use coord1 by default.. too bad if you use this code over very long distances.
	 */
	degreeWidth = degreeHight * cosf(coord1->decLat*0.017);

	/*
	 * Calculate distance.
	 */
	diffLat		= (coord1->decLat - coord2->decLat)*degreeHight;
	diffLong	= (coord1->decLong - coord2->decLong)*degreeWidth;
	dist = sqrtf(diffLat*diffLat + diffLong*diffLong);

	return dist;
}

static float EM411Tan(float y, float x)
{
	float angle=0.0;

	/*
	 * Calculate angle between 0, pi/2 (0 - 90')
	 */
	angle=atan2f(fabs(y),fabs(x));

	/*
	 * Adjust angle for quadrant it's in
	 */
	if(x < 0 && y >= 0) {	/* quadrant 2 */
		angle = M_PI - angle;
	}
	else if(x <= 0 && y < 0) {
		angle += M_PI;
	}
	else if(x > 0 && y < 0) {
		angle = M_PI*2 - angle;
	}

	return angle;
}


#if 0
/*
 * Test the problematic degrees with the arc angles
 * being 0' 90' 180' 270'
 */
void  EM411TestTan(void)
{
	float angle=0.0;
	// 0'
	angle = EM411Tan(0, 1);
	UARTPrintf("Angle should 0' result %f'\r\n", angle/(360*2*M_PI));

	// 45'
	angle = EM411Tan(1, 1);
	UARTPrintf("Angle should 45' result %f'\r\n", angle/(2*M_PI)*360);

	// 90'
	angle = EM411Tan(1, 0);
	UARTPrintf("Angle should 90' result %f'\r\n", angle/(2*M_PI)*360);

	// 135'
	angle = EM411Tan(1, -1);
	UARTPrintf("Angle should 135' result %f'\r\n", angle/(2*M_PI)*360);

	// 180'
	angle = EM411Tan(0, -1);
	UARTPrintf("Angle should 180' result %f'\r\n", angle/(2*M_PI)*360);

	// 225'
	angle = EM411Tan(-1, -1);
	UARTPrintf("Angle should 225' result %f'\r\n", angle/(2*M_PI)*360);

	// 270'
	angle = EM411Tan(-1, 0);
	UARTPrintf("Angle should 270' result %f'\r\n", angle/(2*M_PI)*360);

	// 315'
	angle = EM411Tan(-1, 1);
	UARTPrintf("Angle should 315' result %f'\r\n", angle/(2*M_PI)*360);

	/*
	 * Sleep to show the slow humans the results.
	 */
	chThdSleepMilliseconds(100000);
}
#endif

/*
 * Calculate the course that should be taken to go from coord1 -> coord2
 * Course is radians.
 */
float EM411GetCourse(EM411Coords *coord1, EM411Coords *coord2)
{
	float angle=0.0;
	float y=0.0;
	float x=0.0;
	float degreeHight = 111319.4; /* a 360th part of the circumvention of the earth. */
	float degreeWidth = 0.0;

	/*
	 * Calculate the width of a degree at a given latitude.
	 * We use coord1 by default.. too bad if you use this code over very long distances.
	 */
	degreeWidth = degreeHight * cosf(coord1->decLat*0.017);

	/*
	 * Get absolute values of deltas
	 */
	y=(coord2->decLat - coord1->decLat)*degreeHight;
	x=(coord2->decLong - coord1->decLong)*degreeWidth;

	angle = EM411Tan(y, x);

	return angle;
}
