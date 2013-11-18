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

#include "fc_uart.h"

#include "fc_bmp085.h"
#include "fc_mpu6050.h"
#include "fc_hmc5883.h"
#include "fc_threads.h"
#include "fc_em411.h"
#include "fc_controller.h"
#include "fc_nrf24l01.h"

/*
 * Accelerometer thread
 */
static WORKING_AREA(PollAccelThreadWA, 256);
static msg_t PollAccelThread(void *arg) {
	chRegSetThreadName("PollAccel");
	(void)arg;
	long timeNow=0;
	long timePrev=0;

	while (TRUE) {
		timePrev = MS2ST(chTimeNow());
		MPU6050Update();
		timeNow = MS2ST(chTimeNow());
		// Update 200 time/second
		chThdSleepMilliseconds(5-(timeNow-timePrev));
	}
	return 0;
}

/*
 * Barometer thread
 */
static WORKING_AREA(PollBaroThreadWA, 512);
static msg_t PollBaroThread(void *arg) {
	chRegSetThreadName("PollBaro");
	(void)arg;
	long timeNow=0;
	long timePrev=0;

	while (TRUE) {
		timePrev = MS2ST(chTimeNow());
		BMP085UpdatePressure();
		timeNow = MS2ST(chTimeNow());
		// Update the pressure 20 times a second
		chThdSleepMilliseconds(50-(timeNow-timePrev));
	}
	return 0;
}

/*
 * Compass thread
 */
static WORKING_AREA(PollCompassThreadWA, 256);
static msg_t PollCompassThread(void *arg) {
	chRegSetThreadName("PollCompass");
	(void)arg;
	long timeNow=0;
	long timePrev=0;
	while (TRUE) {
		/*chThdSleepMilliseconds(rand() & 31);*/
		timePrev = MS2ST(chTimeNow());
		HMC5883Update();
		timeNow = MS2ST(chTimeNow());
		chThdSleepMilliseconds(7-(timeNow-timePrev));
	}
	return 0;
}

/*
 * GPS thread
 */
static WORKING_AREA(PollGPSThreadWA, 1280);
static msg_t PollGPSThread(void *arg) {
	chRegSetThreadName("PollGPS");
	(void)arg;
	while (TRUE) {
		/* 
		 * No time out here, as interface is blocking
		 */
		EM411Update();
	}
	return 0;
}

/*
 * Controller thread
 */
static WORKING_AREA(PollControlThreadWA, 256);
static msg_t PollControlThread(void *arg) {
	chRegSetThreadName("PollController");
	(void)arg;
	long timeNow=0;
	long timePrev=0;
	while (TRUE) {
		/*chThdSleepMilliseconds(rand() & 31);*/
		timePrev = MS2ST(chTimeNow());
		CONTROLUpdate();
		timeNow = MS2ST(chTimeNow());
		chThdSleepMilliseconds(2-(timeNow-timePrev));
	}
	return 0;
}


/*
 * NRF thread
 */
static WORKING_AREA(NRFThreadWA, 512);
static msg_t NRFThread(void *arg) {
	chRegSetThreadName("NRFThread");
	(void)arg;
	while (TRUE) {
		NRFHandleIrq();
	}
	return 0;
}

/*
 * Start Threads
 */
void startThreads(void)
{
	/* 
	 * Create barometer thread 
	 */
	chThdCreateStatic(PollBaroThreadWA,
			sizeof(PollBaroThreadWA),
			NORMALPRIO,
			PollBaroThread,
			NULL);

	/* 
	 * Create accelerometer thread 
	 */
	chThdCreateStatic(PollAccelThreadWA,
			sizeof(PollAccelThreadWA),
			NORMALPRIO,
			PollAccelThread,
			NULL);

	/*
	 * Create compass thread
	 */
	chThdCreateStatic(PollCompassThreadWA,
			sizeof(PollCompassThreadWA),
			NORMALPRIO,
			PollCompassThread,
			NULL);

	/*
	 * Create gps thread
	 */
	chThdCreateStatic(PollGPSThreadWA,
			sizeof(PollGPSThreadWA),
			NORMALPRIO,
			PollGPSThread,
			NULL);

	/*
	 * Create control thread
	 */
	chThdCreateStatic(PollControlThreadWA,
			sizeof(PollControlThreadWA),
			NORMALPRIO,
			PollControlThread,
			NULL);

	/*
	 * Create NRF thread
	 */
	chThdCreateStatic(NRFThreadWA,
			sizeof(NRFThreadWA),
			NORMALPRIO,
			NRFThread,
			NULL);
}
