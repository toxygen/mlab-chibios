//
//  microusb.h
//  
//
//  Created by Marian Such on 7/5/13.
//
//

#ifndef _microusb_h
#define _microusb_h

#include "ch.h"
#include "hal.h"
#include "usb_cdc.h"

/*
 * USB Driver structure.
 */
extern SerialUSBDriver SDU1;

int init_usb(void);

#endif
