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

bool TSL2591_Init(void);

void TSL2591_Write(uint8_t reg, uint8_t value);
uint8_t TSL2591_Read(uint8_t reg);
void TSL2591_ReadMulti(uint8_t reg, uint8_t *buf, uint8_t len);
void TSL2591_ReadChannels(uint16_t *ch0, uint16_t *ch1);
int32_t TSL2591_CalcLuxX10(uint16_t ch0, uint16_t ch1);

// ============================ SHT31 =============================
#define SHT31_ADDR (0x44 << 1)
#define SHT31_MEAS_CMD_LOW 0x06U
#define SHT31_MEAS_CMD_HIGH 0x2CU

void SHT31_MeasureCommand(void);
void SHT31_ReadSensor(uint8_t *buf);
void SHT31_ConvertFromRaw(uint8_t *buf, uint32_t *h, int32_t *t);

// =========================== BME280 =============================
#define BME280_ADDR (0x76 << 1)

/* Registers */
#define BME280_REG_ID       0xD0
#define BME280_REG_RESET    0xE0
#define BME280_REG_CTRL_HUM 0xF2
#define BME280_REG_STATUS   0xF3
#define BME280_REG_CTRL_MEAS 0xF4
#define BME280_REG_CONFIG   0xF5
#define BME280_REG_PRESS_MSB 0xF7

#define BME280_REG_CALIB00  0x88
#define BME280_REG_CALIB26  0xE1

#define BME280_RESET_VALUE  0xB6

typedef struct {
	/* Temperature calibration */
	uint16_t dig_T1;
	int16_t dig_T2;
	int16_t dig_T3;

	/* Pressure calibration */
	uint16_t dig_P1;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;

	/* Humidity calibration */
	uint8_t dig_H1;
	int16_t dig_H2;
	uint8_t dig_H3;
	int16_t dig_H4;
	int16_t dig_H5;
	int8_t dig_H6;
} BME280_CalibData;

uint8_t BME280_Init(void);
uint8_t BME280_ReadRaw(int32_t *raw_temp, int32_t *raw_press, int32_t *raw_hum);
int32_t BME280_CompensatePressure(int32_t adcP, int32_t adcT);

#endif /* INC_SENSORS_H_ */
