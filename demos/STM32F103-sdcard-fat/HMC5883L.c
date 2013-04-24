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

#define HMC5883L_addr  0x1E /* 7 bit address */

/* buffers */
static uint8_t rx_data[RX_DEPTH];
static uint8_t tx_data[TX_DEPTH];

static i2cflags_t errors = 0;

static int16_t data_x = 0;
static int16_t data_y = 0;
static int16_t data_z = 0;

/**
 * Init function. Here we will also start personal serving thread.
 */
int init_HMC5883L(void)
{
	msg_t status = RDY_OK;
	systime_t tmo = MS2ST(4);
	
	/* configure magnetometer */
	tx_data[0] = HMC5883L_MODE_REG; /* register address */
	tx_data[1] = 0x00;
	
	/* sending */
	i2cAcquireBus(&I2CD2);
	status = i2cMasterTransmitTimeout(&I2CD2, HMC5883L_addr, tx_data, 2, rx_data, 0, tmo);
	i2cReleaseBus(&I2CD2);
	
	if (status != RDY_OK)
	{
		errors = i2cGetErrors(&I2CD2);
	}
	
	return 0;
}

int request_data_HMC5883L(BaseChannel *chp)
{
	int x, y, z;
	msg_t status = RDY_OK;
	systime_t tmo = MS2ST(4);
	
	chprintf(chp, "idem citat\r\n");
	chThdSleepMilliseconds(500);
	
	tx_data[0] = HMC5883L_OUT_DATA; /* register address */
	i2cAcquireBus(&I2CD2);
	status = i2cMasterReceiveTimeout(&I2CD2, HMC5883L_addr, rx_data, 6, tmo);
	i2cReleaseBus(&I2CD2);
	
	chprintf(chp, "docital som\r\n");
	chThdSleepMilliseconds(500);
	
	if (status != RDY_OK)
	{
		chprintf(chp, "mame tu error\r\n");
		errors = i2cGetErrors(&I2CD2);
	}
	
	chprintf(chp, "pocitam vysledok\r\n");
	x = rx_data[0] + (rx_data[1] << 8);
	y = rx_data[2] + (rx_data[3] << 8);
	z = rx_data[4] + (rx_data[5] << 8);
	return x;
}



