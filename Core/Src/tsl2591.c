/*
 * tsl2591.c
 *
 *  Created on: Dec 9, 2025
 *      Author: Cameron
 */

#include "tsl2591.h"

#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

uint32_t gCplScaled = 0;


// Read a single byte
static uint8_t read_reg(uint8_t reg, uint8_t *buf) {
	uint8_t cmd = TSL2591_CMD_NORMAL | reg;
	return HAL_I2C_Mem_Read(&hi2c1, TSL2591_ADDR, cmd, I2C_MEMADD_SIZE_8BIT, buf, 1, 25);
}

// read multiple bytes
static uint8_t read_multi_reg(uint8_t reg, uint8_t *buf, uint16_t len) {
	uint8_t cmd = TSL2591_CMD_NORMAL | reg;
	return HAL_I2C_Mem_Read(&hi2c1, TSL2591_ADDR, cmd, I2C_MEMADD_SIZE_8BIT, buf, len, 25);
}

// write single byte
static uint8_t write_reg(uint8_t reg, uint8_t val) {
	uint8_t cmd = TSL2591_CMD_NORMAL | reg;
	return HAL_I2C_Mem_Write(&hi2c1, TSL2591_ADDR, cmd, I2C_MEMADD_SIZE_8BIT, &val, 1, 25);
}

// Call this only when integration time or gain are changed
static void update_cpl_scaled(uint8_t controlReg, uint32_t GA) {
	uint32_t atimeMs;
	switch(controlReg & 0x07) {
	case 0: atimeMs = 100; break;
	case 1: atimeMs = 200; break;
	case 2: atimeMs = 300; break;
	case 3: atimeMs = 400; break;
	case 4: atimeMs = 500; break;
	case 5: atimeMs = 600; break;
	default: atimeMs = 100; break;
	}

	uint32_t again;
	switch((controlReg >> 4) & 0x03) {
	case 0: again = 1; break; 		// low
	case 1: again = 25; break;		// med
	case 2: again = 428; break;		// high
	default: again = 9876; break;	// max
	}

	if(GA == 0) GA = 1;

	uint32_t denom = GA * TSL2591_LUX_DF;
	uint32_t num = atimeMs * again * TSL2591_SCALE;
	gCplScaled = (denom != 0) ? (num / denom) : 0;
}

bool TSL2591_Init(void) {
	// check for id
	HAL_StatusTypeDef ret;

	uint8_t id;
	ret = read_reg(TSL2591_REG_ID, &id);
	if(ret != HAL_OK) {
		return false;
	}
	if(id != TSL2591_ID_VALUE) {
		return false;
	}

	write_reg(TSL2591_REG_ENABLE, TSL2591_ENABLE_PON | TSL2591_ENABLE_AEN);
	write_reg(TSL2591_REG_CONTROL, TSL2591_AGAIN_MED | TSL2591_ATIME_100MS);

	ret = read_reg(TSL2591_REG_CONTROL, &id); // reuse id to store atime and again
	if(ret != HAL_OK) {
		return false;
	}

	update_cpl_scaled(id, 1);

	return true;
}

void TSL2591_ReadChannels(uint16_t *ch0, uint16_t *ch1) {
	uint8_t buf[4];
	read_multi_reg(TSL2591_REG_C0DATAL, buf, sizeof(buf));

	// Data received in little endian
	*ch0 = ((uint16_t)buf[1] << 8) | buf[0];
	*ch1 = ((uint16_t)buf[3] << 8) | buf[2];
}

int32_t TSL2591_CalcIntLux(uint16_t ch0, uint16_t ch1) {
	if(gCplScaled == 0) return 0;

	int32_t num1 = (int32_t)ch0 * TSL2591_SCALE - (int32_t)TSL2591_LUX_COEFB * ch1;
	int32_t num2 = (int32_t)TSL2591_LUX_COEFC * ch0 - (int32_t)TSL2591_LUX_COEFD * ch1;

	int32_t lux1 = num1 / (int32_t)gCplScaled;
	int32_t lux2 = num2 / (int32_t)gCplScaled;

	int32_t lux = (lux1 > lux2) ? lux1 : lux2;
	if(lux < 0) lux = 0;

	return lux;
}
