//
//  spi.c
//  
//
//  Created by Marian Such on 1/12/13.
//
//
#include "microspi.h"

static const SPIConfig hs_spicfg = {
	NULL,
	GPIOA,
	GPIOA_SPI1NSS,
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
	palSetPadMode(GPIOA, GPIOA_SPI1NSS, PAL_MODE_OUTPUT_PUSHPULL);
	palSetPad(GPIOA, GPIOA_SPI1NSS);
	return 0;
}

uint8_t writeByteSPI(uint8_t txbyte)
{
	uint8_t rxbyte = 0;
	spiAcquireBus(&SPID1);              /* Acquire ownership of the bus.    */
    spiStart(&SPID1, &hs_spicfg);       /* Setup transfer parameters.       */
    spiSelect(&SPID1);                  /* Slave Select assertion.          */
    spiExchange(&SPID1, 1,
                &txbyte, &rxbyte);          /* Atomic transfer operations.      */
    spiUnselect(&SPID1);                /* Slave Select de-assertion.       */
    spiReleaseBus(&SPID1);
	chThdSleepMilliseconds(1);
	return rxbyte;
}
