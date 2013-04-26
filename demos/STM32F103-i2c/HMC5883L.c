//
//  HMC5883L.c
//  test
//
//  Created by Marian Such on 10/12/12.
//  Copyright (c) 2012. All rights reserved.
//

#include <stdlib.h>

#include "ch.h"
#include "hal.h"

#include "HMC5883L.h"

/**
 * Init function. Here we will also start personal serving thread.
 */
i2cflags_t init_HMC5883L(void)
{
	uint8_t rx_data[RX_DEPTH];
	uint8_t tx_data[TX_DEPTH];
	
	msg_t status = RDY_OK;
	systime_t tmo = MS2ST(4);
	
	/* configure magnetometer */
	tx_data[0] = 0x00; /* register address */
	tx_data[1] = 0x70; /* average of 8 samples */
	
	/* sending */
	i2cAcquireBus(&I2CD2);
	status = i2cMasterTransmitTimeout(&I2CD2, HMC5883L_addr,
									  tx_data, 2, rx_data, 0, tmo);
	i2cReleaseBus(&I2CD2);
	
	if (status != RDY_OK) {
		return i2cGetErrors(&I2CD2);
	}
	
	tx_data[0] = 0x01; /* register address */
	tx_data[1] = 0x20; /* set high gain */
	
	i2cAcquireBus(&I2CD2);
	status = i2cMasterTransmitTimeout(&I2CD2, HMC5883L_addr,
									  tx_data, 2, rx_data, 0, tmo);
	i2cReleaseBus(&I2CD2);
	
	if (status != RDY_OK) {
		return i2cGetErrors(&I2CD2);
	}

	tx_data[0] = 0x02; /* register address */
	tx_data[1] = 0x00; /* continuous measurement */

	i2cAcquireBus(&I2CD2);
	status = i2cMasterTransmitTimeout(&I2CD2, HMC5883L_addr,
									  tx_data, 2, rx_data, 0, tmo);
	i2cReleaseBus(&I2CD2);
	
	if (status != RDY_OK) {
		return i2cGetErrors(&I2CD2);
	}

	return 0;
}

i2cflags_t read_data_HMC5883L(HMC5883L_result * dest)
{
	uint8_t rx_data[RX_DEPTH];
	uint8_t tx_data[TX_DEPTH];

	msg_t status = RDY_OK;
	systime_t tmo = MS2ST(4);
	
	tx_data[0] = 0x06;
	
	i2cAcquireBus(&I2CD2);
	status = i2cMasterReceiveTimeout(&I2CD2, HMC5883L_addr, rx_data, 6, tmo);
	i2cReleaseBus(&I2CD2);
	
	if (status != RDY_OK) {
		return i2cGetErrors(&I2CD2);
	}
	
	tx_data[0] = 0x03;
	
	i2cAcquireBus(&I2CD2);
	status = i2cMasterTransmitTimeout(&I2CD2, HMC5883L_addr, tx_data,
									  1, rx_data, 0, tmo);
	i2cReleaseBus(&I2CD2);
	
	chThdSleepMilliseconds(67);
	
	if (status != RDY_OK) {
		return i2cGetErrors(&I2CD2);
	}
	
	dest->x = (rx_data[0] << 8) | rx_data[1];
	dest->y = (rx_data[2] << 8) | rx_data[3];
	dest->z = (rx_data[4] << 8) | rx_data[5];
	
	/* If overflown, make result invalid (set lower gain /higher GN#/) */
	if (
		( dest->x < -2048) || (dest->x > 2047) ||
		( dest->y < -2048) || (dest->y > 2047) ||
		( dest->z < -2048) || (dest->z > 2047)
		)
	{
		dest->valid = 0;
	}
	else
	{
		dest->valid = 1;
	}
	
	return I2CD_NO_ERROR;
}