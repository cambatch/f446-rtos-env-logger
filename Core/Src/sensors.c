/*
 * tsl2591.c
 *
 *  Created on: Dec 2, 2025
 *      Author: Cameron
 */

#include "sensors.h"

bool TSL2591Init(void) {
	uint8_t id = TSL2591Read(TSL2591_REG_ID);
	if(id != 0x50) {
		return false;
	}

	TSL2591Write(TSL2591_REG_ENABLE, TSL2591_ENABLE_PON);

	HAL_Delay(5);

	TSL2591Write(TSL2591_REG_ENABLE, TSL2591_ENABLE_PON | TSL2591_ENABLE_AEN);

	TSL2591Write(TSL2591_REG_CONTROL, TSL2591_AGAIN_MED | TSL2591_ATIME_100MS);

	return true;
}


void TSL2591Write(uint8_t reg, uint8_t value) {
	uint8_t buf[2];
	buf[0] = TSL2591_CMD_NORMAL | reg;
	buf[1] = value;

	HAL_I2C_Master_Transmit(&hi2c1, TSL2591_ADDR, buf, 2, HAL_MAX_DELAY);
}

uint8_t TSL2591Read(uint8_t reg) {
	uint8_t cmd = TSL2591_CMD_NORMAL | reg;
	uint8_t value;

	HAL_I2C_Master_Transmit(&hi2c1, TSL2591_ADDR, &cmd, 1, HAL_MAX_DELAY);

	HAL_I2C_Master_Receive(&hi2c1, TSL2591_ADDR, &value, 1, HAL_MAX_DELAY);

	return value;
}

void TSL2591ReadMulti(uint8_t reg, uint8_t *buf, uint8_t len) {
	uint8_t cmd = TSL2591_CMD_NORMAL | reg;

	HAL_I2C_Master_Transmit(&hi2c1, TSL2591_ADDR, &cmd, 1, HAL_MAX_DELAY);

	HAL_I2C_Master_Receive(&hi2c1, TSL2591_ADDR, buf, len, HAL_MAX_DELAY);
}

void TSL2591ReadChannels(uint16_t *ch0, uint16_t *ch1) {
	uint8_t buf[4];

	TSL2591ReadMulti(TSL2591_REG_C0DATAL, buf, 4);

	// little endian (low byte first)
	*ch0 = ((uint16_t)buf[1] << 8) | buf[0];
	*ch1 = ((uint16_t)buf[3] << 8) | buf[2];
}

int32_t TSL2591CalcLuxX10(uint16_t ch0, uint16_t ch1) {
	if(ch0 == 0 || ch0 == TSL2591_SATURATED || ch1 == TSL2591_SATURATED) {
		return -1;
	}

	int32_t c0 = ch0;
	int32_t c1 = ch1;

	int32_t diff = c0 - c1;
	if(diff < 0) {
		diff = 0;
	}

	// lux10 = diff^2 * 4080 / (c0 * 2500)
	uint64_t num = (uint64_t)diff * (uint64_t)diff * 4080ull;
	uint64_t den = (uint64_t)c0 * 2500ull;
	if(den == 0) {
		return -1;
	}

	uint64_t lux10_u = num / den;
	if(lux10_u > 0x7FFFFFFF) {
		lux10_u = 0x7FFFFFFF;
	}

	return (int32_t)lux10_u;
}

// ============================ SHT31 ===================================

void SHT31MeasureCommand(void) {
	static const uint8_t cmd[2] = { SHT31_MEAS_CMD_HIGH, SHT31_MEAS_CMD_LOW };

	HAL_I2C_Master_Transmit(&hi2c1, SHT31_ADDR, (uint8_t*)cmd, 2, HAL_MAX_DELAY);
}

// The size of buf MUST be at least 6 bytes
void SHT31ReadSensor(uint8_t *buf) {
	HAL_I2C_Master_Receive(&hi2c1, SHT31_ADDR, buf, 6, HAL_MAX_DELAY);
}

// Assuming pointer to raw sensor data
void SHT31ConvertFromRaw(uint8_t *buf, uint32_t *h, int32_t *t) {
	uint16_t rawTemp = ((uint16_t)buf[0] << 8) | buf[1];
	uint16_t rawRH = ((uint16_t)buf[3] << 8) | buf[4];

	*h = ((uint32_t)1000  * rawRH + 32767u) / 65535;
	*t = -450 + ((int32_t)1750 * rawTemp + 32767) / 65535;
}
