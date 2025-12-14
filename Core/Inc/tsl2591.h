/*
 * tsl2591.h
 *
 *  Created on: Dec 9, 2025
 *      Author: Cameron
 */

#ifndef INC_TSL2591_H_
#define INC_TSL2591_H_

#include <stdint.h>
#include <stdbool.h>

#define TSL2591_ADDR        (0x29 << 1)

// [cmd(7)][transaction(6..5)][addr/sf(4..0)]
#define TSL2591_CMD_NORMAL  0xA0  // cmd=1, transaction=1

// Register addresses
#define TSL2591_REG_ENABLE   		0x00
#define TSL2591_REG_CONTROL  		0x01
#define TSL2591_REG_INT_THRESHOLD 	0x04 // 0x04 - 0x0B
#define TSL2591_REG_PERSIST			0x0C
#define TSL2591_REG_PID				0x11
#define TSL2591_REG_ID       		0x12
#define TSL2591_REG_STATUS			0x13
#define TSL2591_REG_C0DATAL  		0x14 // 0x14 - 0x17

#define TSL2591_ID_VALUE     		0x50

// ENABLE bits
#define TSL2591_ENABLE_PON   0x01
#define TSL2591_ENABLE_AEN   0x02
#define TSL2591_ENABLE_NPIEN 0x80
#define TSL2591_ENABLE_SAI   0x40
#define TSL2591_ENABLE_AIEN  0x10


// CONTROL fields
#define TSL2591_AGAIN_MED    (0x01 << 4) // bits 5:4
#define TSL2591_ATIME_100MS  0x00        // bits 2:0

#define TSL2591_SATURATED 0xFFFFu

// Used to calculate lux
#define TSL2591_SCALE 		100
#define TSL2591_LUX_DF 		408
#define TSL2591_LUX_COEFB 	164
#define TSL2591_LUX_COEFC 	59
#define TSL2591_LUX_COEFD 	86


typedef struct {
	bool ok;
	uint16_t ch1;
	uint16_t ch2;
} TSL2591_SensorData_t;

typedef enum {
	TSL2591_INT_NONE,
	TSL2591_INT_NP,
	TSL2591_INT_P
} TSL2591_Interrupt_t;


bool TSL2591_Init(TSL2591_Interrupt_t interrupt);
void TSL2591_ReadChannels(uint16_t *ch0, uint16_t *ch1);
int32_t TSL2591_CalcIntLux(uint16_t ch0, uint16_t ch1);

#endif /* INC_TSL2591_H_ */
