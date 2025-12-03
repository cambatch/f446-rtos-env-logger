/*
 * tsl2591.h
 *
 *  Created on: Dec 2, 2025
 *      Author: Cameron
 */

#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_

#include <stdbool.h>

#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;


// ========================== TSL2591 ===========================
#define TSL2591_ADDR        (0x29 << 1)
#define TSL2591_CMD_NORMAL  0xA0  // CMD=1, TRANSACTION=01

// Register addresses
#define TSL2591_REG_ENABLE   0x00
#define TSL2591_REG_CONTROL  0x01
#define TSL2591_REG_ID       0x12

// ENABLE bits
#define TSL2591_ENABLE_PON   0x01
#define TSL2591_ENABLE_AEN   0x02

// CONTROL fields
#define TSL2591_AGAIN_MED    (0x01 << 4) // bits 5:4
#define TSL2591_ATIME_100MS  0x00        // bits 2:0

#define TSL2591_REG_STATUS   0x13
#define TSL2591_REG_C0DATAL  0x14

#define TSL2591_SATURATED 0xFFFFu


bool TSL2591Init(void);

void TSL2591Write(uint8_t reg, uint8_t value);
uint8_t TSL2591Read(uint8_t reg);
void TSL2591ReadMulti(uint8_t reg, uint8_t *buf, uint8_t len);
void TSL2591ReadChannels(uint16_t *ch0, uint16_t *ch1);
int32_t TSL2591CalcLuxX10(uint16_t ch0, uint16_t ch1);

// ============================ SHT31 =============================
#define SHT31_ADDR (0x44 << 1)
#define SHT31_MEAS_CMD_LOW 0x06U
#define SHT31_MEAS_CMD_HIGH 0x2CU


void SHT31MeasureCommand(void);
void SHT31ReadSensor(uint8_t *buf);
void SHT31ConvertFromRaw(uint8_t *buf, uint32_t *h, int32_t *t);


#endif /* INC_SENSORS_H_ */
