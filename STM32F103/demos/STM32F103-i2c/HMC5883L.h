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

#define RX_DEPTH 6
#define TX_DEPTH 8

#define HMC5883L_addr       0x1E /* 7 bit address */
#define HMC5883L_CTRL_REG1  0x00
#define HMC5883L_OUT_DATA   0x03
#define HMC5883L_STATUS_REG 0x09

typedef struct HMC5883L_result HMC5883L_result;

struct HMC5883L_result {
	int16_t x;
	int16_t y;
	int16_t z;
	bool_t valid;
};

inline i2cflags_t init_HMC5883L(void);
inline i2cflags_t read_data_HMC5883L(HMC5883L_result * dest);


#endif
