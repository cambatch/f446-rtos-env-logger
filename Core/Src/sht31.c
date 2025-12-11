/*
 * sht31.c
 *
 *  Created on: Dec 10, 2025
 *      Author: Cameron
 */

#include "sht31.h"

#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

bool SHT31_Init(void) {
	HAL_StatusTypeDef ret;
	uint8_t cmd[2] = { SHT31_CMD_SOFT_RESET_HIGH, SHT31_CMD_SOFT_RESET_LOW };

	ret = HAL_I2C_Master_Transmit(&hi2c1, SHT31_ADDR, cmd, 2, 25);
	if(ret != HAL_OK) {
		return false;
	}

	HAL_Delay(10);

	return true;
}

// SHT31 needs 20ms to finish a measurement.
bool SHT31_StartMeasurement(void) {
	HAL_StatusTypeDef ret;
	uint8_t cmd[2] = { SHT31_MEAS_CMD_HIGH, SHT31_MEAS_CMD_LOW };

	ret = HAL_I2C_Master_Transmit(&hi2c1, SHT31_ADDR, cmd, 2, 25);
	return ret == HAL_OK;
}

// Call this >= 20ms after calling SHT31_StartMeasurement
// We assume, a buffer of 6 bytes is supplied.
bool SHT31_ReadSensor(uint8_t *buf) {
	HAL_StatusTypeDef ret;
	ret = HAL_I2C_Master_Receive(&hi2c1, SHT31_ADDR, buf, 6, 25);
	return ret == HAL_OK;
}

// We are assuming supplied pointers are valid...
void SHT31_ConvertFromRaw(uint8_t *buf, uint32_t *h, int32_t *t) {
	uint16_t rawTemp = ((uint16_t)buf[0] << 8) | buf[1];
	    uint16_t rawRH   = ((uint16_t)buf[3] << 8) | buf[4];

	    // Humidity: 0..1000 ‰
	    uint32_t rh = ((uint32_t)1000 * rawRH + 32767u) / 65535u;
	    if (rh > 1000u) rh = 1000u;
	    *h = rh;

	    // Temperature: milli °C = -45000 + 175000 * rawTemp / 65535
	    int32_t t_mC = -45000
	                 + (int32_t)(((uint64_t)175000 * rawTemp + 32767u) / 65535u);
	    *t = t_mC;
}
