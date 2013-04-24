//
//  MLBA_MAG01A.c
//  test
//
//  Created by Marian Such on 10/12/12.
//  Copyright (c) 2012. All rights reserved.
//

#include "ch.h"
#include "hal.h"

#include "MLAB_MAG01A.h"

/* I2C2 */
static const I2CConfig i2cfg1 = {
    OPMODE_I2C,
    400000,
    STD_DUTY_CYCLE,
};



void I2CInit_pns(BaseChannel * chp)
{
	chprintf(chp, "zvnutra idem volat i2cinit\r\n");
	chThdSleepMilliseconds(200);
	i2cInit();
	chprintf(chp, "i2cinit dovolany\r\n");
	chThdSleepMilliseconds(200);
	
	chprintf(chp, "nastavujem porty\r\n");
	chThdSleepMilliseconds(200);
	/* tune ports for I2C2 */
	palSetPadMode(GPIOB, GPIOB_SCL, PAL_MODE_STM32_ALTERNATE_OPENDRAIN);
	palSetPadMode(GPIOB, GPIOB_SDA, PAL_MODE_STM32_ALTERNATE_OPENDRAIN);
	chprintf(chp, "porty nastavene\r\n");
	chThdSleepMilliseconds(200);

	
	chprintf(chp, "idem volat i2cstart %d %d\r\n", &I2CD2, &i2cfg1);
	chThdSleepMilliseconds(200);
	i2cStart(&I2CD2, &i2cfg1, chp);
	chprintf(chp, "i2cstart dovolany\r\n");
	chThdSleepMilliseconds(200);
		
	/* startups. Pauses added just to be safe */
	chThdSleepMilliseconds(100);
	chprintf(chp, "idem volat init hmc5883l\r\n");
	chThdSleepMilliseconds(200);
	init_HMC5883L();
	chprintf(chp, "hmc5883l init dovolany dovolany\r\n");
	chThdSleepMilliseconds(200);
}
