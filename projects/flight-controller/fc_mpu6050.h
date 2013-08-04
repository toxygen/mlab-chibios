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
 * These values differ for every MPU6050 Unit.
 * Please calibrate again when when first using with a new sensor.
 * These ranges are set for a range of accel +-4g and gyro +-500'/s
 */
#define FC_MPU6050_X_ACCEL	517
#define FC_MPU6050_Y_ACCEL	33
#define FC_MPU6050_Z_ACCEL	(8007-8192)  /* 1g down is 8192 */
#define FC_MPU6050_X_GYRO		18
#define FC_MPU6050_Y_GYRO		-93
#define FC_MPU6050_Z_GYRO		-196

/*
 * Declaring an union for the registers and the axis values.
 * The byte order does not match the byte order of
 * the compiler and AVR chip.
 * The AVR chip (on the Arduino board) has the Low Byte
 * at the lower address.
 * But the MPU-6050 has a different order: High Byte at
 * lower address, so that has to be corrected.
 * The register part "reg" is only used internally,
 * and are swapped in code.
 */
typedef union 
{
	struct
	{
		uint8_t x_accel_h;
		uint8_t x_accel_l;
		uint8_t y_accel_h;
		uint8_t y_accel_l;
		uint8_t z_accel_h;
		uint8_t z_accel_l;
		uint8_t t_h;
		uint8_t t_l;
		uint8_t x_gyro_h;
		uint8_t x_gyro_l;
		uint8_t y_gyro_h;
		uint8_t y_gyro_l;
		uint8_t z_gyro_h;
		uint8_t z_gyro_l;
	} reg;
	struct
	{
		int16_t x_accel;
		int16_t y_accel;
		int16_t z_accel;
		int16_t temperature;
		int16_t x_gyro;
		int16_t y_gyro;
		int16_t z_gyro;
	} value;
} accel_t_gyro_union;

typedef struct 
{
	int16_t x_accel;
	int16_t y_accel;
	int16_t z_accel;
	int16_t x_gyro;
	int16_t y_gyro;
	int16_t z_gyro;
} accel_gyro_calibration;

typedef struct
{
	// Output angles in degrees
	float x_angle; // Pitch
	float y_angle; // Roll
	float z_angle;

	// Accelometer data raw (for now)
	int16_t x_accel;
	int16_t y_accel;
	int16_t z_accel;
} MPU6050OutputData;


/*
 * Initialize the MPU6050 sensor.
 */
int MPU6050Init(void);

#if 0
/*
 * Read the gyroscope values from the MPU6050 sensor.
 */
uint8_t MPU6050GetValues(accel_t_gyro_union *accel_t_gyro);
#endif

/*
 * Get the accelerometer and gyro calibration values.
 * This is done by placing the device on a flat leveled surface, and not move it.
 * The routine will take 80 samples and avarage them, returning the results
 */
accel_gyro_calibration *MPU6050GetCalibrationValues(void);

/*
 * This function should only be called by the update thread.
 */
void MPU6050Update(void);

/*
 * Copy the output struct to other threads
 */
void MPU6050GetData(MPU6050OutputData *data);

