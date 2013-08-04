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
#include <stdlib.h>

#include "fc_mpu6050.h"
#include "fc_uart.h"
#include "fc_i2c.h"
#include "fc_kalman.h"
#include "fc_hmc5883.h"


/*
 * Debug Settings
 */
#define FC_MPU6050_DEBUG

/*
 * Register names according to the datasheet.
 * According to the InvenSense document
 * "MPU-6000 and MPU-6050 Register Map
 * and Descriptions Revision 3.2", there are no registers
 * at 0x02 ... 0x18, but according other information
 * the registers in that unknown area are for gain
 * and offsets.
 */
#define MPU6050_AUX_VDDIO          0x01   // R/W
#define MPU6050_SMPLRT_DIV         0x19   // R/W
#define MPU6050_CONFIG             0x1A   // R/W
#define MPU6050_GYRO_CONFIG        0x1B   // R/W
#define MPU6050_ACCEL_CONFIG       0x1C   // R/W
#define MPU6050_FF_THR             0x1D   // R/W
#define MPU6050_FF_DUR             0x1E   // R/W
#define MPU6050_MOT_THR            0x1F   // R/W
#define MPU6050_MOT_DUR            0x20   // R/W
#define MPU6050_ZRMOT_THR          0x21   // R/W
#define MPU6050_ZRMOT_DUR          0x22   // R/W
#define MPU6050_FIFO_EN            0x23   // R/W
#define MPU6050_I2C_MST_CTRL       0x24   // R/W
#define MPU6050_I2C_SLV0_ADDR      0x25   // R/W
#define MPU6050_I2C_SLV0_REG       0x26   // R/W
#define MPU6050_I2C_SLV0_CTRL      0x27   // R/W
#define MPU6050_I2C_SLV1_ADDR      0x28   // R/W
#define MPU6050_I2C_SLV1_REG       0x29   // R/W
#define MPU6050_I2C_SLV1_CTRL      0x2A   // R/W
#define MPU6050_I2C_SLV2_ADDR      0x2B   // R/W
#define MPU6050_I2C_SLV2_REG       0x2C   // R/W
#define MPU6050_I2C_SLV2_CTRL      0x2D   // R/W
#define MPU6050_I2C_SLV3_ADDR      0x2E   // R/W
#define MPU6050_I2C_SLV3_REG       0x2F   // R/W
#define MPU6050_I2C_SLV3_CTRL      0x30   // R/W
#define MPU6050_I2C_SLV4_ADDR      0x31   // R/W
#define MPU6050_I2C_SLV4_REG       0x32   // R/W
#define MPU6050_I2C_SLV4_DO        0x33   // R/W
#define MPU6050_I2C_SLV4_CTRL      0x34   // R/W
#define MPU6050_I2C_SLV4_DI        0x35   // R  
#define MPU6050_I2C_MST_STATUS     0x36   // R
#define MPU6050_INT_PIN_CFG        0x37   // R/W
#define MPU6050_INT_ENABLE         0x38   // R/W
#define MPU6050_INT_STATUS         0x3A   // R  
#define MPU6050_ACCEL_XOUT_H       0x3B   // R  
#define MPU6050_ACCEL_XOUT_L       0x3C   // R  
#define MPU6050_ACCEL_YOUT_H       0x3D   // R  
#define MPU6050_ACCEL_YOUT_L       0x3E   // R  
#define MPU6050_ACCEL_ZOUT_H       0x3F   // R  
#define MPU6050_ACCEL_ZOUT_L       0x40   // R  
#define MPU6050_TEMP_OUT_H         0x41   // R  
#define MPU6050_TEMP_OUT_L         0x42   // R  
#define MPU6050_GYRO_XOUT_H        0x43   // R  
#define MPU6050_GYRO_XOUT_L        0x44   // R  
#define MPU6050_GYRO_YOUT_H        0x45   // R  
#define MPU6050_GYRO_YOUT_L        0x46   // R  
#define MPU6050_GYRO_ZOUT_H        0x47   // R  
#define MPU6050_GYRO_ZOUT_L        0x48   // R  
#define MPU6050_EXT_SENS_DATA_00   0x49   // R  
#define MPU6050_EXT_SENS_DATA_01   0x4A   // R  
#define MPU6050_EXT_SENS_DATA_02   0x4B   // R  
#define MPU6050_EXT_SENS_DATA_03   0x4C   // R  
#define MPU6050_EXT_SENS_DATA_04   0x4D   // R  
#define MPU6050_EXT_SENS_DATA_05   0x4E   // R  
#define MPU6050_EXT_SENS_DATA_06   0x4F   // R  
#define MPU6050_EXT_SENS_DATA_07   0x50   // R  
#define MPU6050_EXT_SENS_DATA_08   0x51   // R  
#define MPU6050_EXT_SENS_DATA_09   0x52   // R  
#define MPU6050_EXT_SENS_DATA_10   0x53   // R  
#define MPU6050_EXT_SENS_DATA_11   0x54   // R  
#define MPU6050_EXT_SENS_DATA_12   0x55   // R  
#define MPU6050_EXT_SENS_DATA_13   0x56   // R  
#define MPU6050_EXT_SENS_DATA_14   0x57   // R  
#define MPU6050_EXT_SENS_DATA_15   0x58   // R  
#define MPU6050_EXT_SENS_DATA_16   0x59   // R  
#define MPU6050_EXT_SENS_DATA_17   0x5A   // R  
#define MPU6050_EXT_SENS_DATA_18   0x5B   // R  
#define MPU6050_EXT_SENS_DATA_19   0x5C   // R  
#define MPU6050_EXT_SENS_DATA_20   0x5D   // R  
#define MPU6050_EXT_SENS_DATA_21   0x5E   // R  
#define MPU6050_EXT_SENS_DATA_22   0x5F   // R  
#define MPU6050_EXT_SENS_DATA_23   0x60   // R  
#define MPU6050_MOT_DETECT_STATUS  0x61   // R  
#define MPU6050_I2C_SLV0_DO        0x63   // R/W
#define MPU6050_I2C_SLV1_DO        0x64   // R/W
#define MPU6050_I2C_SLV2_DO        0x65   // R/W
#define MPU6050_I2C_SLV3_DO        0x66   // R/W
#define MPU6050_I2C_MST_DELAY_CTRL 0x67   // R/W
#define MPU6050_SIGNAL_PATH_RESET  0x68   // R/W
#define MPU6050_MOT_DETECT_CTRL    0x69   // R/W
#define MPU6050_USER_CTRL          0x6A   // R/W
#define MPU6050_PWR_MGMT_1         0x6B   // R/W
#define MPU6050_PWR_MGMT_2         0x6C   // R/W
#define MPU6050_FIFO_COUNTH        0x72   // R/W
#define MPU6050_FIFO_COUNTL        0x73   // R/W
#define MPU6050_FIFO_R_W           0x74   // R/W
#define MPU6050_WHO_AM_I           0x75   // R
 
/*
 * Default I2C address for the MPU-6050
 */
//#define MPU6050_I2C_ADDRESS 0b1101001
#define MPU6050_I2C_ADDRESS 0b1101000
//#define MPU6050_I2C_ADDRESS 0x68

/*
 * According to the data sheet the device should return 
 * 0x68 as reply to Who am I.
 */
#define MPU6050_WHO_AM_I_REPLY 0x68

/*
 * Set I2C time-out to 4 milli seconds.
 */
systime_t tmo = MS2ST(4);		/* 4 milliseconds */

/*
 * Mutex locking multi threaded variables.
 */
static Mutex					mpu6050mtx; /* Mutex */

/*
 * Structure to store the integrated gyro angles.
 */
MPU6050OutputData mpu6050_out;

/*
 * Kalman structures holding the variables for the axis.
 */
static kalmanStruct xKalman;
static kalmanStruct yKalman;
static kalmanStruct zKalman;

/*
 * Stores the previous cycle time, in order to calculate the dt.
 */
static long timePrev=0;

/*
 * Calibration structure
 */
accel_gyro_calibration MPU6050CalValues;

/*
 * Swap all high and low bytes.
 * After this, the registers values are swapped,
 * so the structure name like x_accel_l does no
 * longer contain the lower byte.
 */
static uint8_t swap;
#define SWAP(x,y) swap = x; x = y; y = swap

/*
 * Read the 'Who am I' and compare to the known value.
 * Return 0 when all okay, when errors occur -1 is returned.
 */
static uint8_t MPU6050GetWhoAmI(void)
{
	//uint8_t retval=0;
	uint8_t rcv_buf[2];
	uint8_t tx_buf;

	tx_buf=MPU6050_FIFO_R_W;
	I2CRead(MPU6050_I2C_ADDRESS, &tx_buf, 1, rcv_buf, 2);

	/*
	 * Compare to expected value
	 */
	if(rcv_buf[1] != MPU6050_WHO_AM_I_REPLY){
#ifdef FC_MPU6050_DEBUG
		UARTPrintf("'Who am I' failed, expected 0x%x, but got 0x%x\r\n", MPU6050_WHO_AM_I_REPLY, *rcv_buf);
#endif
		/*
		 * Error state...
		 */
		return -1;	
	} else {
#ifdef FC_MPU6050_DEBUG
		UARTPrintf("MPU6050 'Who am I' successful.\r\n");
#endif
	}

	return 0;
}

/*
 * Reset MPU6050
 */
static void MPU6050Reset(void)
{
	uint8_t retval=0;
	uint8_t tx_buf[2];

	// Nothing here for now
	tx_buf[0]=MPU6050_PWR_MGMT_1;
	tx_buf[1]=0x80;

	retval = I2CWrite(MPU6050_I2C_ADDRESS, tx_buf, 2);
#ifdef FC_MPU6050_DEBUG
	if(retval != RDY_OK) {
		UARTPrintf("MPU6050_PWR_MGMT_1 I2C Error\r\n");
	}
#endif
	chThdSleepMilliseconds(10);
}

/*
 * Check the state the Sensor is in.
 * change when it's the wrong state.
 */
static void MPU6050SetSleepState(void)
{
	uint8_t retval=0;
	uint8_t tx_buf[2];

	/*
	 * Set clock source to gyroscope 1
	 */
	tx_buf[0]=MPU6050_PWR_MGMT_1;
	tx_buf[1]=0x01;

	retval = I2CWrite(MPU6050_I2C_ADDRESS, tx_buf, 2);
#ifdef FC_MPU6050_DEBUG
	if(retval != RDY_OK) {
		UARTPrintf("MPU6050_PWR_MGMT_1 I2C Error\r\n");
	}
#endif
}

/*
 * Set sample rate to 400kHz
 */
void MPU6050SetSampleRate(void)
{
	uint8_t retval=0;
	uint8_t tx_buf[2];

	// Nothing here for now
	tx_buf[0]=MPU6050_SMPLRT_DIV;
	tx_buf[1]=0x13; // 8kHz => 19 (dec) devider = 400Hz

	retval = I2CWrite(MPU6050_I2C_ADDRESS, tx_buf, 2);
#ifdef FC_MPU6050_DEBUG
	if(retval == RDY_OK) {
		UARTPrintf("MPU6050_SMPLRT_DIV I2C Error\r\n");
	}
#endif
}


/*
 * Set Gyro range 250'/s
 */
void MPU6050SetGyroRange(void)
{
	uint8_t retval=0;
	uint8_t tx_buf[2];

	/*
	 * Most sensitive setting 250'/s.
	 */
	tx_buf[0]=MPU6050_GYRO_CONFIG;
	tx_buf[1]=0x08;
	//I2CWrite(MPU6050_I2C_ADDRESS, tx_buf, 2);
	retval = I2CWrite(MPU6050_I2C_ADDRESS, tx_buf, 2);

#ifdef FC_MPU6050_DEBUG
	if(retval != RDY_OK) {
		UARTPrintf("MPU6050_GYRO_CONFIG I2C Error\r\n");
	}
#endif
	chThdSleepMilliseconds(10);
}

/*
 * Set accelerometer range 2g range
 */
void MPU6050SetAccelRange(void)
{
	uint8_t retval=0;
	uint8_t tx_buf[2];

	//MPU6050_ACCEL_CONFIG
	tx_buf[0]=MPU6050_ACCEL_CONFIG;
	tx_buf[1]=0x08;
	retval = I2CWrite(MPU6050_I2C_ADDRESS, tx_buf, 2);

#ifdef FC_MPU6050_DEBUG
	if(retval != RDY_OK) {
		UARTPrintf("MPU6050_ACCEL_CONFIG I2C Error\r\n");
	}
#endif
	chThdSleepMilliseconds(10);
}

/*
 * Initialize the Calibration values for the MPU6050
 */
static void MPU6050InitCalibration(void)
{
	MPU6050CalValues.x_accel = FC_MPU6050_X_ACCEL;
	MPU6050CalValues.y_accel = FC_MPU6050_Y_ACCEL;
	MPU6050CalValues.z_accel = FC_MPU6050_Z_ACCEL;
	MPU6050CalValues.x_gyro	 = FC_MPU6050_X_GYRO;
	MPU6050CalValues.y_gyro	 = FC_MPU6050_Y_GYRO;
	MPU6050CalValues.z_gyro  = FC_MPU6050_Z_GYRO;
}

/*
 * Initialize the MPU6050 sensor.
 */
int MPU6050Init(void)
{
	uint8_t retval=0;

	/*
	 * Reset the device.
	 */
	MPU6050Reset();

	/*
	 * Check if device is present
	 */
	retval = MPU6050GetWhoAmI();

	/*
	 * Set sleep state
	 */
	MPU6050SetSleepState();

	/*
	 * Set Gyro range 250'/s
	 */
	MPU6050SetGyroRange();

	/*
	 * Set accelerometer range 2g range
	 */
	MPU6050SetAccelRange();

	/*
	 * Set sample rate to 400Hz
	 */
	MPU6050SetSampleRate();

	/*
	 * Setup the calibration values for the MPU6050
	 */
	MPU6050InitCalibration();

	/*
	 * Set all output data to 0
	 */
	memset(&mpu6050_out, 0, sizeof(MPU6050OutputData));

	/*
	 * Initialize the kalman structures
	 */
	KALInit(&xKalman);
	KALInit(&yKalman);
	KALInit(&zKalman);

	/*
	 * Initialize Mutex
	 */
	chMtxInit(&mpu6050mtx); /* Mutex initialization before use */
	
	return retval;
}

/*
 * Read the gyroscope values from the MPU6050 sensor.
 */
uint8_t MPU6050GetValues(accel_t_gyro_union *accel_t_gyro)
{
	uint8_t retval=0;
	uint8_t tx_buf[1];

	tx_buf[0]=MPU6050_ACCEL_XOUT_H;       
	retval = I2CRead(MPU6050_I2C_ADDRESS, tx_buf, 1, (uint8_t *) accel_t_gyro, (uint8_t) sizeof(accel_t_gyro_union));

#ifdef FC_MPU6050_DEBUG
	if(retval != RDY_OK) {
		UARTPrintf("MPU6050_ACCEL_XOUT_H I2C Error\r\n");
	}
#endif

	/*
	 * Fix the byte ordering
	 */
	SWAP (accel_t_gyro->reg.x_accel_h, accel_t_gyro->reg.x_accel_l);
	SWAP (accel_t_gyro->reg.y_accel_h, accel_t_gyro->reg.y_accel_l);
	SWAP (accel_t_gyro->reg.z_accel_h, accel_t_gyro->reg.z_accel_l);
	SWAP (accel_t_gyro->reg.t_h, accel_t_gyro->reg.t_l);
	SWAP (accel_t_gyro->reg.x_gyro_h, accel_t_gyro->reg.x_gyro_l);
	SWAP (accel_t_gyro->reg.y_gyro_h, accel_t_gyro->reg.y_gyro_l);
	SWAP (accel_t_gyro->reg.z_gyro_h, accel_t_gyro->reg.z_gyro_l);

	/*
	 * Subtract the calibration values.
	 */
	accel_t_gyro->value.x_accel -= MPU6050CalValues.x_accel;
	accel_t_gyro->value.y_accel -= MPU6050CalValues.y_accel;
	accel_t_gyro->value.z_accel -= MPU6050CalValues.z_accel;
	accel_t_gyro->value.x_gyro -= MPU6050CalValues.x_gyro;
	accel_t_gyro->value.y_gyro -= MPU6050CalValues.y_gyro;
	accel_t_gyro->value.z_gyro -= MPU6050CalValues.z_gyro;

	return retval;
}

/*
 * Get the accelerometer and gyro calibration values.
 * This is done by placing the device on a flat leveled surface, and not move it.
 * The routine will take 80 samples and average them, returning the results
 */
accel_gyro_calibration *MPU6050GetCalibrationValues(void)
{
	accel_t_gyro_union accel_t_gyro;
	int count=0;
	int x_accel=0;
	int y_accel=0;
	int z_accel=0;
	int x_gyro=0;
	int y_gyro=0;
	int z_gyro=0;

	/*
	 * Clear current values.
	 */
	memset(&accel_t_gyro, 0, sizeof(accel_t_gyro_union));

	/*
	 * Do measurements
	 */
	for(count=0; count<80; count++) {
		MPU6050GetValues(&accel_t_gyro);
		x_accel += accel_t_gyro.value.x_accel;
		y_accel += accel_t_gyro.value.y_accel;
		z_accel += accel_t_gyro.value.z_accel;
		x_gyro  += accel_t_gyro.value.x_gyro;
		y_gyro	 += accel_t_gyro.value.y_gyro;
		z_gyro	 += accel_t_gyro.value.z_gyro;

		chThdSleepMilliseconds(10);
	}

	/*
	 * Store the averages
	 */
	MPU6050CalValues.x_accel = x_accel/80;
	MPU6050CalValues.y_accel = y_accel/80;
	MPU6050CalValues.z_accel = z_accel/80;
	MPU6050CalValues.x_gyro	 = x_gyro/80;
	MPU6050CalValues.y_gyro	 = y_gyro/80;
	MPU6050CalValues.z_gyro	 = z_gyro/80;

	return &MPU6050CalValues;
}

/*
 * Calculate total acceleration
 */
static float MPU6050GetTotalAccel(accel_t_gyro_union *accel_t_gyro)
{
	float total;

	total = sqrtf(accel_t_gyro->value.x_accel*accel_t_gyro->value.x_accel + 
			accel_t_gyro->value.y_accel*accel_t_gyro->value.y_accel + 
			accel_t_gyro->value.z_accel*accel_t_gyro->value.z_accel);

	return total;
}

/* 
 * Use the total acceleration vector length 
 * In combination with the angle acceleration,
 * To calculate the angle of acceleration for the given 
 * The angle is returned in radians.
 * !This calculation only works for 0 to PI degrees. 
 */
static float MPU6050GetAccelAngle(int16_t x_accel, float total_accel)
{
	float angle=0.0;

	angle = acosf(abs(x_accel)/total_accel);

	/*
	 * When the x < 0 2PI - angle
	 */
	if(x_accel < 0){
		angle = M_PI - angle;
	}

	return angle;
}

void MPU6050Update(void)
{
	long  timeNow=0;									/* Time now */	
	long  timeDelta=0;								/* Delta t */
	float totalAccel=0;
	float xAccelAngle=0.0;						/* X angle calculated from accelerometer */
	float yAccelAngle=0.0;						/* Y angle calculated from accelerometer */
	float Heading=0.0;
	accel_t_gyro_union newData;			/* New Data */

	/*
	 * Get a new set of measurements
	 */
	MPU6050GetValues(&newData);

	/*
	 * Calculate total acceleration vector.
	 * Because the X-offset actually provides the Y rotation and vice-versa.
	 * That is why the X and Y are switched in the code below.
	 */
	totalAccel = MPU6050GetTotalAccel(&newData);
	xAccelAngle = MPU6050GetAccelAngle(newData.value.y_accel*-1, totalAccel);
	yAccelAngle = MPU6050GetAccelAngle(newData.value.x_accel, totalAccel);

	/*
	 * Calculate time difference
	 * This is in milliseconds.
	 */
	timeNow = MS2ST(chTimeNow());
	timeDelta=timeNow-timePrev;
	timePrev=timeNow;

	/*
	 * Integrate the data with the values we already have
	 */
	chMtxLock(&mpu6050mtx);

	/*
	 * calculate the amount of movement in radiants
	 * Multiply them by the delta time
	 * Full range is 8.72665Rad/s over 16bit 2 complement.
	 * Delta time is in milliseconds
	 */
	//mpu6050_out.x_angle += newData.value.x_gyro*8.72665/32768*timeDelta/1000.0;
	//mpu6050_out.y_angle += newData.value.y_gyro*8.72665/32768*timeDelta/1000.0;
	//mpu6050_out.z_angle += newData.value.z_gyro*8.72665/32768*timeDelta/1000.0;

	// Just add the accelerometer data.
	mpu6050_out.x_accel = newData.value.x_accel;
	mpu6050_out.y_accel = newData.value.y_accel;
	mpu6050_out.z_accel = newData.value.z_accel;

	// Use Kalman to get the X and Y angles without drift Using the accelerometer
	mpu6050_out.x_angle = KALCalculate(&xKalman, xAccelAngle, newData.value.x_gyro*8.72665/32768, timeDelta); 
	mpu6050_out.y_angle = KALCalculate(&yKalman, yAccelAngle, newData.value.y_gyro*8.72665/32768, timeDelta); 

	// Use the compass heading to calculate the Z-angle relative to the magnetic north.
	// Because the kalman filter can push the heading below 0, so locking the result between 0 and 2PI
	Heading=HMC5883GetHeading();
	mpu6050_out.z_angle = fmod(KALCalculate(&zKalman, Heading, newData.value.z_gyro*8.72665/32768, timeDelta) , (2*M_PI));

	chMtxUnlock();
}

/*
 * Copy the output structure to other threads
 */
void MPU6050GetData(MPU6050OutputData *data)
{
	chMtxLock(&mpu6050mtx);
	memcpy(data, &mpu6050_out, sizeof(MPU6050OutputData));
	chMtxUnlock();
}

