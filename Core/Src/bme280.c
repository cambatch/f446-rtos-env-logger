/*
 * bme280.c
 *
 *  Created on: Dec 14, 2025
 *      Author: Cameron
 */

#include "bme280.h"

#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;
static BME280_CalibData_t calib;


static uint8_t write_reg(uint8_t reg, uint8_t value) {
	return HAL_I2C_Mem_Write(&hi2c1, BME280_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
			&value, 1, 25);
}

static uint8_t read_reg(uint8_t reg, uint8_t *buf, uint16_t len) {
	return HAL_I2C_Mem_Read(&hi2c1, BME280_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf,
			len, 25);
}

static uint8_t read_calibration(void) {
	uint8_t buf1[26];
	uint8_t buf2[7];
	uint8_t ret;

	ret = read_reg(BME280_REG_CALIB00, buf1, sizeof(buf1));
	if (ret != HAL_OK)
		return ret;

	calib.digT1 = (uint16_t) (buf1[1] << 8 | buf1[0]);
	calib.digT2 = (int16_t) (buf1[3] << 8 | buf1[2]);
	calib.digT3 = (int16_t) (buf1[5] << 8 | buf1[4]);

	calib.digP1 = (uint16_t) (buf1[7] << 8 | buf1[6]);
	calib.digP2 = (int16_t) (buf1[9] << 8 | buf1[8]);
	calib.digP3 = (int16_t) (buf1[11] << 8 | buf1[10]);
	calib.digP4 = (int16_t) (buf1[13] << 8 | buf1[12]);
	calib.digP5 = (int16_t) (buf1[15] << 8 | buf1[14]);
	calib.digP6 = (int16_t) (buf1[17] << 8 | buf1[16]);
	calib.digP7 = (int16_t) (buf1[19] << 8 | buf1[18]);
	calib.digP8 = (int16_t) (buf1[21] << 8 | buf1[20]);
	calib.digP9 = (int16_t) (buf1[23] << 8 | buf1[22]);

	calib.digH1 = buf1[25]; // 0xA1

	ret = read_reg(BME280_REG_CALIB26, buf2, sizeof(buf2));
	if (ret != HAL_OK)
		return ret;

	calib.digH2 = (int16_t) (buf2[1] << 8 | buf2[0]);
	calib.digH3 = buf2[2];
	calib.digH4 = (int16_t) ((buf2[3] << 4) | (buf2[4] & 0x0F));
	calib.digH5 = (int16_t) ((buf2[5] << 4) | (buf2[4] >> 4));
	calib.digH6 = (int8_t) buf2[6];

	return HAL_OK;
}

static uint8_t wait_for_measure() {
	uint8_t status;
	do {
		if (read_reg(BME280_REG_STATUS, &status, 1) != HAL_OK)
			return HAL_ERROR;
	} while (status & 0x08);

	return HAL_OK;
}

uint8_t BME280_Init() {
	HAL_StatusTypeDef ret;
	uint8_t id;

	ret = read_reg(BME280_REG_ID, &id, 1);
	if (ret != HAL_OK)
		return ret;
	if (id != 0x60)
		return HAL_ERROR;

	ret = write_reg(BME280_REG_RESET, BME280_RESET_VALUE);
	if (ret != HAL_OK)
		return ret;
	HAL_Delay(5);

	ret = read_calibration();
	if (ret != HAL_OK)
		return ret;

	// 0x01 => humidity oversampling = x1
	ret = write_reg(BME280_REG_CTRL_HUM, 0x01);
	if (ret != HAL_OK)
		return ret;

	// 0xA8 => tstandby = 1000ms, filter = 16
	ret = write_reg(BME280_REG_CONFIG, 0xA8);
	if (ret != HAL_OK)
		return ret;

	// 0x2F => pressure oversampling = x1, temperature oversampling = x1, mode = normal
	ret = write_reg(BME280_REG_CTRL_MEAS, 0x2F);
	if (ret != HAL_OK)
		return ret;

	ret = wait_for_measure();
	return ret;
}

uint8_t BME280_ReadRaw(int32_t *raw_temp, int32_t *raw_press, int32_t *raw_hum) {
	uint8_t data[8];
	HAL_StatusTypeDef ret;

	ret = wait_for_measure();
	if (ret != HAL_OK)
		return ret;

	// Burst read from 0xF7: press[3], temp[3], hum[2]
	ret = read_reg(BME280_REG_PRESS_MSB, data, sizeof(data));
	if (ret != HAL_OK)
		return ret;

	int32_t adc_P = ((int32_t) data[0] << 12) | ((int32_t) data[1] << 4)
			| ((int32_t) data[2] >> 4);

	int32_t adc_T = ((int32_t) data[3] << 12) | ((int32_t) data[4] << 4)
			| ((int32_t) data[5] >> 4);

	int32_t adc_H = ((int32_t) data[6] << 8) | (int32_t) data[7];

	if (raw_press)
		*raw_press = adc_P;
	if (raw_temp)
		*raw_temp = adc_T;
	if (raw_hum)
		*raw_hum = adc_H;

	return HAL_OK;
}

int32_t BME280_CompensatePressure(int32_t adcP, int32_t adcT) {
	int32_t var1, var2, t_fine;
	uint32_t p;

	// temperature calculation
	var1 = ((((adcT >> 3) - ((int32_t) calib.digT1 << 1)))
			* ((int32_t) calib.digT2)) >> 11;

	var2 = (((((adcT >> 4) - ((int32_t) calib.digT1))
			* ((adcT >> 4) - ((int32_t) calib.digT1))) >> 12)
			* ((int32_t) calib.digT3)) >> 14;

	t_fine = var1 + var2;

	// Pressure calculation
	var1 = (((int32_t) t_fine) >> 1) - (int32_t) 64000;
	var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t) calib.digP6);
	var2 = var2 + ((var1 * ((int32_t) calib.digP5)) << 1);
	var2 = (var2 >> 2) + (((int32_t) calib.digP4) << 16);
	var1 = (((((int32_t) calib.digP3) * (((var1 >> 2) * (var1 >> 2)) >> 13))
			>> 3) + ((((int32_t) calib.digP2) * var1) >> 1)) >> 18;
	var1 = ((32768 + var1) * ((int32_t) calib.digP1)) >> 15;

	if (var1 == 0)
		return 0; // avoid division by zero

	p = (((uint32_t) 1048576 - (uint32_t) adcP) - (var2 >> 12)) * 3125;

	if (p < 0x80000000)
		p = (p << 1) / (uint32_t) var1;
	else
		p = (p / (uint32_t) var1) * 2;

	var1 = (((int32_t) calib.digP9) * (int32_t) (((p >> 3) * (p >> 3)) >> 13))
			>> 12;

	var2 = (((int32_t) (p >> 2)) * ((int32_t) calib.digP8)) >> 13;

	p = (uint32_t) ((int32_t) p + ((var1 + var2 + calib.digP7) >> 4));

	return (int32_t) p; // Return Pa
}
