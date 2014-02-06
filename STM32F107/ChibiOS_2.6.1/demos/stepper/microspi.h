//
//  microspi.h
//  
//
//  Created by Marian Such on 1/12/13.
//
//

#ifndef _microspi_h
#define _microspi_h

#include "ch.h"
#include "hal.h"

#define GPIOA_SPI1NSS 4

#define STEPPER1_CS 4
#define STEPPER2_CS 5

int initSPI(void);
uint8_t writeByteSPI(uint8_t txbyte);

void selectStepper(int num);

#endif
