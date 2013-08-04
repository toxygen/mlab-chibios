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

/*
 * Debug Settings
 */
#define FC_I2C_DEBUG 1

/* Initialize I2C to 400 kHz mode. */
static const I2CConfig i2cfg1 = {
    OPMODE_I2C,
    400000,
    //FAST_DUTY_CYCLE_16_9, /* Changed this when moving from Chibios 2.4.1 to 2.4.2 */
    FAST_DUTY_CYCLE_2,
};

#if 0
static const I2CConfig i2cfg1 = {
    OPMODE_I2C,
    100000,
    STD_DUTY_CYCLE,
};
#endif


/*
 * Initialize the I2C subsystem, including the pins.
 */
void I2CInit(void){
	i2cInit();

	i2cStart(&I2CD1, &i2cfg1);

	/* tune ports for I2C1*/
	palSetPadMode(IOPORT2, 6, PAL_MODE_STM32_ALTERNATE_OPENDRAIN);
	palSetPadMode(IOPORT2, 7, PAL_MODE_STM32_ALTERNATE_OPENDRAIN);

	/* startups. Pauses added just to be safe */
	chThdSleepMilliseconds(100);
}

/*
 * Read a number of bytes into a provided buffer.
 */
msg_t I2CRead(i2caddr_t dev_addr, uint8_t *tx_buf, uint8_t tx_size, uint8_t *rx_buf, uint8_t rx_size)
{
	msg_t retval=0;						/* Return value */
	systime_t tmo = MS2ST(4);		/* 4 milliseconds */
	i2cflags_t i2cflags=0;

	/*
   * Claim I2C bus for this thread
	 */
  i2cAcquireBus(&I2CD1);

	/*
	 * Read the different elements, 
	 */
	retval = i2cMasterTransmitTimeout (
			&I2CD1,
			dev_addr,
			tx_buf,
			tx_size,
			rx_buf,
			rx_size,
			tmo);

	/*
	 * Release I2C bus
	 */
  i2cReleaseBus(&I2CD1);

#ifdef FC_I2C_DEBUG
	if (retval != RDY_OK) {
		i2cflags = i2cGetErrors(&I2CD1);
		switch(i2cflags){
			case I2CD_ACK_FAILURE:
				UARTPrintf("I2C Error ACK Failure\r\n");
				break;
			case I2CD_TIMEOUT:
				UARTPrintf("I2C Error Timeout\r\n");
				break;
			default:
				UARTPrintf("I2C Unknown error\r\n");
		}	
	}
#endif
	
	return retval;
}

/*
 * Get short from I2C bus.
 * Because of byte ordering reverse MSB and LSB
 */
msg_t I2CReadShort(i2caddr_t dev_addr, uint8_t *tx_buf, uint8_t tx_size, short *result)
{
	uint8_t	rx_buf[2];					/* Receive buffer */
	msg_t retval=0;

	/*
	 * Get value.
	 */
	retval = I2CRead(dev_addr, tx_buf, tx_size, rx_buf, 2);

	/*
	 * When results are read, reverse LSB and MSB.
	 */
	if (retval == RDY_OK) {
		*result = (short) (rx_buf[0] << 8) + rx_buf[1];
	} 

	return retval;
}

/*
 * Read a number of bytes into a provided buffer.
 */
msg_t I2CWrite(i2caddr_t dev_addr, uint8_t *tx_buf, uint8_t tx_size)
{
	msg_t retval=0;

	/*
	 * Sneaky do a read :)
	 */
	retval = I2CRead(dev_addr, tx_buf, tx_size, NULL, 0);

	return retval;
}
