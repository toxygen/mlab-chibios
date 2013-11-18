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

#include "fc_bmp085.h"
#include "fc_uart.h"
#include "fc_kalman.h"
#include "fc_i2c.h"
#include "fc_gen.h"

#include <math.h>

/*
 * Structures holding sensor data.
 */
static BMP085CalData	bmp085caldata;
static BMP085Temp			bmp085temp;
static long						bmp085p0;
static long						bmp085prev;
static long						bmp085cur=0;
static uint8_t				bmp085new=0;
static Mutex					bmp085mtx; /* Mutex */


/*
 * Debug Settings
 */
//#define FC_BMP085_DEBUG
//#define FC_BMP085_CALIBRATE

/*
 * Settings for driver
 */
//#define FC_BMP085_ADDR 0b1110111
#define FC_BMP085_ADDR 0xEE>>1
#define FC_BMP085_OSS	 3

/*
 * Get the calibration data.
 * The pointer is reused, so don't free the returned pointer.
 */
msg_t BMP085GetCalData(void)
{
	msg_t retval=0;						/* Return value */
	uint8_t addr=0;

	/*
	 * Read the different elements, 
	 */
	addr=0xAA;
	retval = I2CReadShort(FC_BMP085_ADDR, &(addr), 1, &(bmp085caldata.ac1));
	addr=0xAC;
	retval &= I2CReadShort(FC_BMP085_ADDR, &(addr), 1, &(bmp085caldata.ac2));
	addr=0xAE;
	retval &= I2CReadShort(FC_BMP085_ADDR, &(addr), 1, &(bmp085caldata.ac3));
	addr=0xB0;
	retval &= I2CReadShort(FC_BMP085_ADDR, &(addr), 1, (short*) &(bmp085caldata.ac4));
	addr=0xB2;
	retval &= I2CReadShort(FC_BMP085_ADDR, &(addr), 1, (short*) &(bmp085caldata.ac5));
	addr=0xB4;
	retval &= I2CReadShort(FC_BMP085_ADDR, &(addr), 1, (short*) &(bmp085caldata.ac6));
	addr=0xB6;
	retval &= I2CReadShort(FC_BMP085_ADDR, &(addr), 1, &(bmp085caldata.b1));
	addr=0xB8;
	retval &= I2CReadShort(FC_BMP085_ADDR, &(addr), 1, &(bmp085caldata.b2));
	addr=0xBA;
	retval &= I2CReadShort(FC_BMP085_ADDR, &(addr), 1, &(bmp085caldata.mb));
	addr=0xBC;
	retval &= I2CReadShort(FC_BMP085_ADDR, &(addr), 1, &(bmp085caldata.mc));
	addr=0xBE;
	retval &= I2CReadShort(FC_BMP085_ADDR, &(addr), 1, &(bmp085caldata.md));
	
#ifdef FC_BMP085_DEBUG
	UARTPrintf("ac1		%hd\r\n", bmp085caldata.ac1);
	UARTPrintf("ac2		%hd\r\n", bmp085caldata.ac2);
	UARTPrintf("ac3		%hd\r\n", bmp085caldata.ac3);
	UARTPrintf("ac4		%hu\r\n", bmp085caldata.ac4);
	UARTPrintf("ac5		%hu\r\n", bmp085caldata.ac5);
	UARTPrintf("ac6		%hu\r\n", bmp085caldata.ac6);
	UARTPrintf("b1		%hd\r\n", bmp085caldata.b1);
	UARTPrintf("b2		%hd\r\n", bmp085caldata.b2);
	UARTPrintf("mb		%hd\r\n", bmp085caldata.mb);
	UARTPrintf("mc		%hd\r\n", bmp085caldata.mc);
	UARTPrintf("md		%hd\r\n", bmp085caldata.md);
#endif

	/*
	 * Return status.
   */
	return retval;
}

/*
 * Get Raw Temperature from the BPM085 sensor
 */
static short BMP085GetRawTemp(void)
{
	uint8_t	send_buf[2];					/* Receive buffer */
	short		temp=0;			/* Temperature */

	/*
	 * Request temperature conversion
	 */
	send_buf[0]=0xF4;
	send_buf[1]=0x2E;
	I2CReadShort(FC_BMP085_ADDR, send_buf, 2, &temp);

	chThdSleepMilliseconds(5);

	/*
	 * Request converted temperature.
	 */
	send_buf[0]=0xF6;
	I2CReadShort(FC_BMP085_ADDR, send_buf, 1, &temp);

	return temp;
}

/*
 * Get the temperature
 */
BMP085Temp *BMP085UpdateTemp(void)
{
	short rawtemp=0;

	/*
   * Get Raw Temperature
	 */
	rawtemp = BMP085GetRawTemp();

	/*
	 * Calculate compensated temperature.
	 */
	bmp085temp.x1 = (rawtemp-bmp085caldata.ac6)*bmp085caldata.ac5/(1<<15);
	bmp085temp.x2 = bmp085caldata.mc*(1<<11)/(bmp085temp.x1+bmp085caldata.md);
	bmp085temp.b5 = bmp085temp.x1+bmp085temp.x2;
	bmp085temp.temp = (bmp085temp.b5+8)/(1<<4);
	
	return &bmp085temp;
}

/*
 * Get Raw pressure reading from BMP085 sensor
 */	
static long BMP085GetRawPressure(void)
{
	uint8_t					rcv_buf[3];					/* Receive buffer */
	uint8_t					send_buf[2];				/* Receive buffer */
	//systime_t tmo = MS2ST(4);						/* 4 milliseconds */
	long		press=0;						/* Temperature */

	/*
	 * Request a conversion
	 */
	send_buf[0]=0xF4;
	send_buf[1]=0x34 +(FC_BMP085_OSS<<6);
	I2CRead(FC_BMP085_ADDR, send_buf, 2, NULL, 0);


	chThdSleepMilliseconds(26);

	/*
	 * Read the result
	 */
	send_buf[0]=0xF6;
	I2CRead(FC_BMP085_ADDR, send_buf, 1, rcv_buf, 3);

	/*
	 * Swap byte order
	 */
	press = ((rcv_buf[0]<<16) + (rcv_buf[1]<<8) + rcv_buf[2]) >> (8-FC_BMP085_OSS);
	
	return press;
}

/*
 * Get the current temperature
 */
short BMP085GetTemp(void)
{
	short temp;

	chMtxLock(&bmp085mtx);
	temp = bmp085temp.temp;
	chMtxUnlock();

	return temp;
}

/*
 * Get the temperature
 */
static long BMP085GetValue(void)
{
	long rawpress=0;
	long press=0;
	long b6, x1, x2, x3, b3; // Some none descript variables
	unsigned long b4, b7;

	/*
	 * get temperature 
	 */
	BMP085UpdateTemp();

	/*
	 * Get Pressure from device
	 */
	rawpress = BMP085GetRawPressure();
#ifdef FC_BMP085_DEBUG
	UARTPrintf("Raw Pressure	%ld\r\n", rawpress);
#endif

#ifdef FC_BMP085_CALIBRATE
	/*
   * Test values from the datasheet 
	 * to verify calculation
	 */
	bmp085caldata.ac1=408;
	bmp085caldata.ac2=-72;
	bmp085caldata.ac3=-14383;
	bmp085caldata.ac4=32741;
	bmp085caldata.ac5=32757;
	bmp085caldata.ac6=23153;
	bmp085caldata.b1=6190;
	bmp085caldata.b2=4;
	bmp085caldata.mb=-32768;
	bmp085caldata.mc=-8711;
	bmp085caldata.md=2868;
	bmp085temp.x1=4743;
	bmp085temp.x2=-2344;
	bmp085temp.b5=2399;
	bmp085temp.temp=150;
	rawpress=23843;
#endif

	/*
	 * Calculate Pressure
	 * The code is taken from the BPM085 datasheet, and not very descriptive..
	 */
	b6 = bmp085temp.b5 - 4000;
	x1 = (bmp085caldata.b2*(b6*b6/(1<<12)))/(1<<11);
	x2 = bmp085caldata.ac2*b6/(1<<11);
	x3 = x1+x2;
#ifdef FC_BMP085_DEBUG
	UARTPrintf("b6						%ld\r\n", b6);
	UARTPrintf("x1						%ld\r\n", x1);
	UARTPrintf("x2						%ld\r\n", x2);
	UARTPrintf("x3						%ld\r\n", x3);
#endif

	b3 = (((bmp085caldata.ac1*4+x3)<<FC_BMP085_OSS)+2)/4;
	x1 = bmp085caldata.ac3*b6/(1<<13);
	x2 = (bmp085caldata.b1*(b6*b6/(1<<12)))/(1<<16);
	x3 = ((x1+x2)+2)/(1<<2);
#ifdef FC_BMP085_DEBUG
	UARTPrintf("b3						%ld\r\n", b3);
	UARTPrintf("x1						%ld\r\n", x1);
	UARTPrintf("x2						%ld\r\n", x2);
	UARTPrintf("x3						%ld\r\n", x3);
#endif

	b4 = bmp085caldata.ac4*(unsigned long)(x3+32768)/(1<<15);
	b7 = ((unsigned long)rawpress-b3)*(50000 >> FC_BMP085_OSS);
	if(b7<0x80000000) {
		press = (b7*2)/b4;
	} else {
		press = (b7/b4)*2;
	}	

#ifdef FC_BMP085_DEBUG
	UARTPrintf("b4						%ld\r\n", b4);
	UARTPrintf("b7						%ld\r\n", b7);
	UARTPrintf("press					%ld\r\n", press);
#endif

	x1 = (press/(1<<8))*(press/(1<<8));
#ifdef FC_BMP085_DEBUG
	UARTPrintf("x1						%ld\r\n", x1);
#endif
	x1 = (x1*3038)/(1<<16);
#ifdef FC_BMP085_DEBUG
	UARTPrintf("x1						%ld\r\n", x1);
#endif
	x2 = (-7357 * press)/(1<<16);
#ifdef FC_BMP085_DEBUG
	UARTPrintf("x2						%ld\r\n", x2);
#endif
	press = press+(x1+x2+3791)/(1<<4);

	return press;
}

/*
 * Get the current pressure, and store it as pressure 0
 */
static void BMP085GetP0(void)
{
	uint8_t count=0;
	long p=0;

	for(count=0; count<10; count++) {
		p += BMP085GetValue();
		
		/*
		 * Wait between samples
		 */
    chThdSleepMilliseconds(100);
	}
	
	bmp085p0 = p/10;
}

/*
 * Update pressure in register
 */
void BMP085UpdatePressure(void)
{
	chMtxLock(&bmp085mtx);
	bmp085cur = BMP085GetValue();
	bmp085new=1;
	chMtxUnlock();
}

/*
 * Get the altitude from the current barometric pressure.
 */
float BMP085GetAltitude(void)
{
	float alt=0.0;
	long	p=0;
	
	/*
	 * Get pressure
	 */
	chMtxLock(&bmp085mtx);
	p=bmp085cur;
	chMtxUnlock();
	//p = BMP085GetValue();

	/*
	 * Calculate Altitude in meters
	 */
	//alt = 44330*pow(1-((float)p/(float)bmp085p0),(1/5.255));
	alt = (bmp085p0-p)/100.0*8.43;
	alt = floorf(alt*10)/10;
	return alt;
}

/*
 * Get the last pressure
 */
long BMP085GetPressure(void)
{
	long retval;
	chMtxLock(&bmp085mtx);
	retval = bmp085cur;
	chMtxUnlock();
	return retval;
}

/*
 * Take 20 samples, use them to calculate the variance in the measurements.
 */
float BMP085GetVariance(void)
{
	int count=0;
	float result=0.0;
	float probes[20]={0.0};
	
	/*
	 * Get 20 measurements
	 */
	for(count=1; count < 20; count++){
		probes[count] = BMP085GetAltitude();
    chThdSleepMilliseconds(100);
	}

	/*
	 * Calculate the BMP085 altitude variance
	 */
	result = GENCalcVariance(probes, count);

	return result;
}


/*
 * Get the altitude compensated using a Kalman filter.
 */
float BMP085GetKalmanAltitude(void)
{
	float alt;

	alt = BMP085GetAltitude();
	if(bmp085new==0) {
		return bmp085prev;
	}
	bmp085new=0;

	/* 
	 * Feed new pressure to Kalman routine.
	 * Use some default values for the noise factors.
	 */
	alt = KALsimple(alt, bmp085prev, 0.222, 0.617);
	
	/*
	 * Predicted value is current value.
	 */
	bmp085prev = alt;

	/*
	 * Since the accuracy in any case won't be better than .1 meter,
	 * floor the number to 1 decimal.
	 */
	alt = floorf(alt*10)/10;
	return alt;
}

/*
 * Initialize the BMP085 driver.
 */
void BMP085Init(void)
{
	/*
	 * Get callibration data
	 */
	BMP085GetCalData();

	/*
	 * Get pressure on altitude 0
	 */
	BMP085GetP0();

	/*
	 * Set the seed value for the kalman filter
	 */
	bmp085prev = bmp085p0;

	/*
	 * Initialize Mutex
	 */
	chMtxInit(&bmp085mtx); /* Mutex initialization before use */

	/*
	 * Set first pressure value.
	 */
	BMP085UpdatePressure();
}
