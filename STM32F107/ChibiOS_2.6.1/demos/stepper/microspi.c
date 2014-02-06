//
//  spi.c
//  
//
//  Created by Marian Such on 1/12/13.
//
//
#include "microspi.h"

static SPIConfig const * spicfg;


static const SPIConfig stepper1_spicfg = {
	NULL,
	GPIOA,
	STEPPER1_CS,
	SPI_CR1_CPHA | SPI_CR1_CPOL | SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0
};

static const SPIConfig stepper2_spicfg = {
	NULL,
	GPIOC,
	STEPPER2_CS,
	SPI_CR1_CPHA | SPI_CR1_CPOL | SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0
};

int initSPI(void)
{
	/*
	 * SPI1 I/O pins setup.
	 */
	palSetPadMode(GPIOA, 5, PAL_MODE_STM32_ALTERNATE_PUSHPULL);     /* SCK. */
	palSetPadMode(GPIOA, 6, PAL_MODE_INPUT);     /* MISO.*/
	palSetPadMode(GPIOA, 7, PAL_MODE_STM32_ALTERNATE_PUSHPULL);     /* MOSI.*/
	palSetPadMode(GPIOA, STEPPER1_CS, PAL_MODE_OUTPUT_PUSHPULL);
	palSetPad(GPIOA, STEPPER1_CS);
	
	palSetPadMode(GPIOC, STEPPER2_CS, PAL_MODE_OUTPUT_PUSHPULL);
	palSetPad(GPIOA, STEPPER2_CS);
	

	
	return 0;
}

void selectStepper(int num)
{
	switch (num) {
		case 1:
			spicfg = &stepper1_spicfg;
			break;
			
		case 2:
			spicfg = &stepper2_spicfg;
			break;
			
		default:
			break;
	}
}

uint8_t writeByteSPI(uint8_t txbyte)
{
	uint8_t rxbyte = 0;
	spiAcquireBus(&SPID1);              /* Acquire ownership of the bus.    */
    spiStart(&SPID1, spicfg);       /* Setup transfer parameters.       */
    spiSelect(&SPID1);                  /* Slave Select assertion.          */
    spiExchange(&SPID1, 1,
                &txbyte, &rxbyte);          /* Atomic transfer operations.      */
    spiUnselect(&SPID1);                /* Slave Select de-assertion.       */
    spiReleaseBus(&SPID1);
	chThdSleepMilliseconds(1);
	return rxbyte;
}
