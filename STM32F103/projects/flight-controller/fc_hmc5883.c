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
#include <string.h>
#include <math.h>

#include "fc_hmc5883.h"
#include "fc_uart.h"
#include "fc_i2c.h"
#include "fc_mpu6050.h"

/*
 * Structure to store compass data.
 */
compass_union compass;

/*
 * Float holding heading in Radiants.
 */
float HMC5883Heading;

/*
 * Mutex locking multi threaded variables.
 */
static Mutex					hmc5883mtx; /* Mutex */

/*
 * Swap all high and low bytes.
 * After this, the registers values are swapped,
 * so the structure name like x_accel_l does no
 * longer contain the lower byte.
 */
static uint8_t swap;
#define SWAP(x,y) swap = x; x = y; y = swap

/*
 * HMC5883 registers
 */
#define HMC5883_CONFIG_REG_A						0x00
#define HMC5883_CONFIG_REG_B						0x01
#define HMC5883_MODE_REG								0x02
#define HMC5883_DATA_OUT_X_MSB_REG			0x03
#define	HMC5883_DATA_OUT_X_LSB_REG			0x04
#define HMC5883_DATA_OUT_Z_MSB_REG			0x05
#define HMC5883_DATA_OUT_Z_LSB_REG			0x06
#define HMC5883_DATA_OUT_Y_MSB_REG      0x07
#define HMC5883_DATA_OUT_Y_LSB_REG      0x08
#define HMC5883_STATUS_REG							0x09
#define HMC5883_IDENT_REG_A							0x0a
#define HMC5883_IDENT_REG_B							0x0b
#define HMC5883_IDENT_REG_C							0x0c

/*
 * Debug Settings
 */
#define FC_HMC5883_DEBUG

#define HMC5883_I2C_ADDRESS 0x3C>>1


/*
 * Set Config register A
 */
static uint8_t HMC5883SetConfigRegA(void)
{
	uint8_t retval=0;
	uint8_t tx_buf[2];

	/* 
	 * Set :
	 * 8 Samples per measurement
	 * 75 samples per second
	 * Normal measuring configuration
	 */
	tx_buf[0]=HMC5883_CONFIG_REG_A;
	tx_buf[1]=0b01111000;

	retval = I2CWrite(HMC5883_I2C_ADDRESS, tx_buf, 2);
#ifdef FC_HMC5883_DEBUG
	if(retval != RDY_OK) {
		UARTPrintf("HMC5883_CONFIG_REG_A I2C Error\r\n");
	}
#endif
	chThdSleepMilliseconds(10);

	return retval;
}

/*
 * Set Config register B
 */
static uint8_t HMC5883SetConfigRegB(void)
{
	uint8_t retval=0;
	uint8_t tx_buf[2];

	/* 
	 * Set :
	 * }- 1.3 Ga Gain
	 */
	tx_buf[0]=HMC5883_CONFIG_REG_B;
	tx_buf[1]=0b00100000;

	retval = I2CWrite(HMC5883_I2C_ADDRESS, tx_buf, 2);
#ifdef FC_HMC5883_DEBUG
	if(retval != RDY_OK) {
		UARTPrintf("HMC5883_CONFIG_REG_B I2C Error\r\n");
	}
#endif
	chThdSleepMilliseconds(10);

	return retval;
}

/*
 * Set Config register B
 */
static uint8_t HMC5883SetModeReg(void)
{
	uint8_t retval=0;
	uint8_t tx_buf[2];

	/* 
	 * Set :
	 * Continuous-Measurement Mode.
	 */
	tx_buf[0]=HMC5883_MODE_REG;
	tx_buf[1]=0b00000000;

	retval = I2CWrite(HMC5883_I2C_ADDRESS, tx_buf, 2);
#ifdef FC_HMC5883_DEBUG
	if(retval != RDY_OK) {
		UARTPrintf("HMC5883_MODE_REG I2C Error\r\n");
	}
#endif
	chThdSleepMilliseconds(10);

	return retval;
}

/*
 * Initialize the BMP085 driver.
 */
uint8_t HMC5883Init(void)
{
	uint8_t retval=0;

	/*
	 * Setup the config registers.
	 */
	retval  = HMC5883SetConfigRegA();
	retval |= HMC5883SetConfigRegB();

	/*
	 * Setup operation mode.
	 */
	retval |= HMC5883SetModeReg();

	/*
	 * Initialize Mutex
	 */
	chMtxInit(&hmc5883mtx); /* Mutex initialization before use */

	return retval;
}

/*
 * Get Compass data
 */
void HMC5883GetData(compass_union *compassPtr)
{
	chMtxLock(&hmc5883mtx);
	memcpy(compassPtr, &compass, sizeof(compass_union));
	chMtxUnlock();
}

/*
 * Calculate compensated heading.
 */
static void HMC5883CalcHeading(void)
{
	float Xh=0.0;
	float Yh=0.0;
	float Pitch=0.0;
	float Roll=0.0;
	MPU6050OutputData MPU6050Data;

	/*
	 * Get Accelerometer data.
	 */
	MPU6050GetData(&MPU6050Data);
	Pitch = MPU6050Data.x_angle - M_PI/2;
	Roll   = MPU6050Data.y_angle - M_PI/2;

	/*
	 * Thanks to https://www.loveelectronics.co.uk/Tutorials/13/tilt-compensated-compass-arduino-tutorial
	 * Xh = XM * cos(Pitch) + ZM * sin(Pitch)
	 * Yh = XM * sin(Roll) * sin(Pitch) + YM * cos(Roll) - ZM * sin(Roll) * cos(Pitch) 
	 * Heading = arctan(Yh/Xh)
	 */
	Xh = compass.value.x_compass * cos(Pitch) + compass.value.z_compass * sin(Pitch);
	Yh = compass.value.x_compass * sin(Roll) * sin(Pitch) + compass.value.y_compass * cos(Roll) - compass.value.z_compass * sin(Roll) * cos(Pitch);

	/*
	 * Get current Yaw and Pitch angles
	 */
	HMC5883Heading = atan2f(Yh, Xh);

	/*
	 * Because the tan gives +Pi to -Pi, convert to 0 to 2Pi
	 */
	if(HMC5883Heading < 0) {
		HMC5883Heading += 2*M_PI;
	}
}

/*
 * Update sensor data.
 * This method should only be called by the update thread.
 */
void HMC5883Update(void)
{
	uint8_t retval=0;
	uint8_t tx_buf[1];

	chMtxLock(&hmc5883mtx);
	tx_buf[0]=HMC5883_DATA_OUT_X_MSB_REG;
	retval = I2CRead(HMC5883_I2C_ADDRESS, tx_buf, 1, (uint8_t *) &compass, (uint8_t) sizeof(compass_union));
	chMtxUnlock();

#ifdef FC_HMC5883_DEBUG
	if(retval != RDY_OK) {
		UARTPrintf("MPU6050_ACCEL_XOUT_H I2C Error\r\n");
	}
#endif

	/*
	 * Fix the byte ordering
	 */
	chMtxLock(&hmc5883mtx);
	SWAP (compass.reg.x_compass_h, compass.reg.x_compass_l);
	SWAP (compass.reg.y_compass_h, compass.reg.y_compass_l);
	SWAP (compass.reg.z_compass_h, compass.reg.z_compass_l);

	/*
	 * Calculate the compass heading compensated for angle offset.
	 */
	HMC5883CalcHeading();
	chMtxUnlock();
}

/*
 * Get heading in radiants.
 */
float HMC5883GetHeading(void)
{
	float Heading;

	chMtxLock(&hmc5883mtx);
	Heading = HMC5883Heading;
	chMtxUnlock();

	return Heading;
}

