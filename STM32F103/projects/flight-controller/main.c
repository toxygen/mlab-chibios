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

#include "stm32f10x.h"
#include "ch.h"
#include "hal.h"


#include "fc_uart.h"
#include "fc_gpio.h"
#include "fc_ext.h"
#include "fc_i2c.h"
#include "fc_spi.h"
#include "fc_bmp085.h"
#include "fc_kalman.h"
#include "fc_mpu6050.h"
#include "fc_hmc5883.h"
#include "fc_threads.h"
#include "fc_em411.h"
#include "fc_controller.h"
#include "fc_pwm.h"
#include "fc_nrf24l01.h"
#include "fc_gen.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

/*
 * Coordinates to my parents house as test :)
 */
#define TEST_LAT	52.492913
#define TEST_LONG	 4.592467

/*
 * Dummy abort symbol, need for newlib
 * Added this to support the official ARM toolchain
 * https://launchpad.net/gcc-arm-embedded
 */
void abort(void){while(1==1){}}

#if 0
/*
 * Execute calibration cycle
 */
static void calibration(void)
{
	accel_gyro_calibration *calivalue;

	/*
	 * Do Calibration
	 */
	calivalue = MPU6050GetCalibrationValues();
	UARTPrintf("Accel X :%hd\r\n", calivalue->x_accel);
	UARTPrintf("Accel Y :%hd\r\n", calivalue->y_accel);
	UARTPrintf("Accel Z :%hd\r\n", calivalue->z_accel);
	UARTPrintf("Gyro  X :%hd\r\n", calivalue->x_gyro);
	UARTPrintf("Gyro  Y :%hd\r\n", calivalue->y_gyro);
	UARTPrintf("Gyro  Z :%hd\r\n", calivalue->z_gyro);

	chThdSleepMilliseconds(100000);
}
#endif

/*
 * Application entry point.
 */
int main(void) {
	//BMP085Temp *temp=NULL;
	//short			  temp=0;
	//long				pressure=0;
	float				alt=0.0;
	//float				heading=0.0;
	//accel_t_gyro_union accel_t_gyro;
	//compass_union compass;
	//MPU6050OutputData gyroData;
	ControlStruct	control;
	EM411FixData gpsFix;
	EM411CourseSpeed gpsCs;
	EM411Coords	here;
	EM411Coords	there;
	PWMoutput		out;
	float dist=0.0;
	float angle=0.0;
	float	variance=0.0;

	/*
	 * Setup coordinates to there place.
	 */
	there.decLat=TEST_LAT;
	there.decLong=TEST_LONG;

	/*
	 * Remapping the PWM out ports before intializing chibios.
	 * Full Remap TIM3 to PC6-9
	 */
	//GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);   
	RCC->APB2ENR |=  1 <<  0;
	
	AFIO->MAPR |= 0x00000C00;		/* Remap Timer 3 to the remap ports */


  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

	/*
	 * Initialize peripherals
	 */
	UARTInit();
	EXTInit();
	PWMInit();
	I2CInit();
	SPIInit();
	CONTROLInit();

	/*
	 * Starts the transmission, it will be handled entirely in background.
	 */
	UARTPrintf("Starting...\r\n");

	/*
	 * Device initialization
	 */
	UARTPrintf("Initializing BMP085\r\n");
	BMP085Init();
	UARTPrintf("Initializing MPU6050\r\n");
	MPU6050Init();
	UARTPrintf("Initializing HMC5883\r\n");
	HMC5883Init();
	UARTPrintf("Initializing EM411\r\n");
	EM411Init();
	UARTPrintf("Initializing NRF24L01\r\n");
	NRFInit();

	/*
	 * Starting threads
	 */
	startThreads();


	/*
	 * Run NRF24L01 test
	 */
	NRFtest();
	//GENTestVariance();
	variance = BMP085GetVariance();
	UARTPrintf("BMP085 variance: %f\r\n", variance);
	chThdSleepMilliseconds(10000);

#if 0
	/*
	 * Provide Sensor calibration values.
	 */
	calibration();
#endif

	/*
	 * Clear screen
	 */
	UARTPrintf( "\033[2J" );


  /*
   * Normal main() thread activity, in this demo it does nothing.
   */
  while (TRUE) {
		/*
		 * Get data
		 */
		//temp = BMP085GetTemp();
		//UARTPrintf("Temp:			%hu\r\n", temp);
		//pressure = BMP085GetPressure();
		//UARTPrintf("Pressure:	%ld\r\n", pressure);
		alt = BMP085GetAltitude();
		UARTPrintf("Altitude:	%f\r\n", alt);
		alt = BMP085GetKalmanAltitude();
		UARTPrintf("Altitude Kalman:	%f\r\n", alt);

#if 0
		memset(&accel_t_gyro, 0, sizeof(accel_t_gyro));
		MPU6050GetValues(&accel_t_gyro);
		UARTPrintf("Acc  X:%6.2f Y:%6.2f Z:%6.2f\r\n", (accel_t_gyro.value.x_accel/8192.0), (accel_t_gyro.value.y_accel/8192.0), (accel_t_gyro.value.z_accel/8192.0));
		UARTPrintf("Gyro X:%6.2f Y:%6.2f Z:%6.2f\r\n", (accel_t_gyro.value.x_gyro/131.0), (accel_t_gyro.value.y_gyro/131.0), (accel_t_gyro.value.z_gyro/131.0));
		UARTPrintf("Temp %hd'C\r\n", accel_t_gyro.value.temperature>>1);

		MPU6050GetData(&gyroData);
		UARTPrintf("Acc  X:%6.2f Y:%6.2f Z:%6.2f\r\n", (gyroData.x_accel/8192.0), (gyroData.y_accel/8192.0), (gyroData.z_accel/8192.0));
		UARTPrintf("Gyro X:%6.2f Y:%6.2f Z:%6.2f\r\n", gyroData.x_angle, gyroData.y_angle, gyroData.z_angle);

		memset(&compass, 0, sizeof(compass));
		HMC5883GetData(&compass);
		heading=HMC5883GetHeading();
		UARTPrintf("Compass  X:%.6hd Y:%.6hd Z:%.6hd Heading:%.6f\r\n", compass.value.x_compass, compass.value.y_compass, compass.value.z_compass, heading);
#endif
		
		// GPS data
		EM411GetGpgga(&gpsFix);
		UARTPrintf("GPS lat: %8f long: %8f sats: %2d %2.2hd:%2.2hd:%2.2hd\r\n", 
				gpsFix.decLat, gpsFix.decLong, gpsFix.satsUsed,
				gpsFix.hour, gpsFix.min, gpsFix.sec);

		EM411GetGpvtg(&gpsCs);
		UARTPrintf("GPS course: %6f speed: %4f\r\n", gpsCs.course, gpsCs.speed);

		// Test distance to point.
		EM411FixToCoord(&gpsFix, &here);
		dist = EM411GetDistance(&here, &there);
		angle = EM411GetCourse(&here, &there);
		UARTPrintf("GPS distance : %6f course %6f\r\n", dist, angle/(2*M_PI)*360);

		// Show the PID controller output.
		CONTROLGetOut(&control);
		UARTPrintf("PID output X:%d Y:%d Z:%d     \r\n", control.x, control.y, control.z);

		// Print Motor Power distribution
		CONTROLGetMotorOut(&out);
		UARTPrintf("%4d   %4d\r\n", out.m1, out.m2);
		UARTPrintf("     X\r\n");
		UARTPrintf("%4d   %4d\r\n", out.m4, out.m3);

#if 0
    //chThdSleepMilliseconds(10);
		ledon2(0);
    chThdSleepMilliseconds(5);
		ledoff2(0);
#endif
		ledtoggle2();
    //chThdSleepMilliseconds(10);
		UARTPrintf( "\033[0;0H" ); // Move cursor to 0,0
  }

 // return 0;
}
