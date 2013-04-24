//
//  HMC5883L.h
//  test
//
//  Created by Marian Such on 10/12/12.
//  Copyright (c) 2012 FIT CVUT. All rights reserved.
//

#ifndef test_HMC5883L_h
#define test_HMC5883L_h

#include "chprintf.h"

#define RX_DEPTH 8
#define TX_DEPTH 8

#define HMC5883L_CTRL_REG1  0x00
#define HMC5883L_MODE_REG   0x02
#define HMC5883L_OUT_DATA   0x03
#define HMC5883L_STATUS_REG 0x09

inline int init_HMC5883L(void);
inline int request_data_HMC5883L(BaseChannel *chp);


#endif
