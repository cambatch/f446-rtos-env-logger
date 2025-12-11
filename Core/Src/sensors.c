/*
 * tsl2591.c
 *
 *  Created on: Dec 2, 2025
 *      Author: Cameron
 */

#include "sensors.h"

static BME280_CalibData_t calib;
static MPU6050_AccelFs_t sAccelFs;
static MPU6050_GyroFs_t sGyroFs;

// ============================ SHT31 ===================================

void SHT31_MeasureCommand(void) {
	static const uint8_t cmd[2] = { SHT31_MEAS_CMD_HIGH, SHT31_MEAS_CMD_LOW };

	HAL_I2C_Master_Transmit(&hi2c1, SHT31_ADDR, (uint8_t*) cmd, 2,
	HAL_MAX_DELAY);
}

// The size of buf MUST be at least 6 bytes
void SHT31_ReadSensor(uint8_t *buf) {
	HAL_I2C_Master_Receive(&hi2c1, SHT31_ADDR, buf, 6, HAL_MAX_DELAY);
}

// Assuming pointer to raw sensor data
void SHT31_ConvertFromRaw(uint8_t *buf, uint32_t *h, int32_t *t) {
	uint16_t rawTemp = ((uint16_t) buf[0] << 8) | buf[1];
	uint16_t rawRH = ((uint16_t) buf[3] << 8) | buf[4];

	*h = ((uint32_t) 1000 * rawRH + 32767u) / 65535;
	*t = -450 + ((int32_t) 1750 * rawTemp + 32767) / 65535;
}

// ============================ BME280 ===================================

static uint8_t BME280_WriteReg(uint8_t reg, uint8_t value) {
	return HAL_I2C_Mem_Write(&hi2c1, BME280_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
			&value, 1, HAL_MAX_DELAY);
}

static uint8_t BME280_ReadReg(uint8_t reg, uint8_t *buf, uint16_t len) {
	return HAL_I2C_Mem_Read(&hi2c1, BME280_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf,
			len, HAL_MAX_DELAY);
}

static uint8_t BME280_ReadCalibration(void) {
	uint8_t buf1[26];
	uint8_t buf2[7];
	uint8_t ret;

	ret = BME280_ReadReg(BME280_REG_CALIB00, buf1, sizeof(buf1));
	if (ret != HAL_OK)
		return ret;

	calib.dig_T1 = (uint16_t) (buf1[1] << 8 | buf1[0]);
	calib.dig_T2 = (int16_t) (buf1[3] << 8 | buf1[2]);
	calib.dig_T3 = (int16_t) (buf1[5] << 8 | buf1[4]);

	calib.dig_P1 = (uint16_t) (buf1[7] << 8 | buf1[6]);
	calib.dig_P2 = (int16_t) (buf1[9] << 8 | buf1[8]);
	calib.dig_P3 = (int16_t) (buf1[11] << 8 | buf1[10]);
	calib.dig_P4 = (int16_t) (buf1[13] << 8 | buf1[12]);
	calib.dig_P5 = (int16_t) (buf1[15] << 8 | buf1[14]);
	calib.dig_P6 = (int16_t) (buf1[17] << 8 | buf1[16]);
	calib.dig_P7 = (int16_t) (buf1[19] << 8 | buf1[18]);
	calib.dig_P8 = (int16_t) (buf1[21] << 8 | buf1[20]);
	calib.dig_P9 = (int16_t) (buf1[23] << 8 | buf1[22]);

	calib.dig_H1 = buf1[25]; // 0xA1

	ret = BME280_ReadReg(BME280_REG_CALIB26, buf2, sizeof(buf2));
	if (ret != HAL_OK)
		return ret;

	calib.dig_H2 = (int16_t) (buf2[1] << 8 | buf2[0]);
	calib.dig_H3 = buf2[2];
	calib.dig_H4 = (int16_t) ((buf2[3] << 4) | (buf2[4] & 0x0F));
	calib.dig_H5 = (int16_t) ((buf2[5] << 4) | (buf2[4] >> 4));
	calib.dig_H6 = (int8_t) buf2[6];

	return HAL_OK;
}

static uint8_t BME280_WaitForMeasDone() {
	uint8_t status;
	do {
		if (BME280_ReadReg(BME280_REG_STATUS, &status, 1) != HAL_OK)
			return HAL_ERROR;
	} while (status & 0x08);

	return HAL_OK;
}

uint8_t BME280_Init() {
	HAL_StatusTypeDef ret;
	uint8_t id;

	ret = BME280_ReadReg(BME280_REG_ID, &id, 1);
	if (ret != HAL_OK)
		return ret;
	if (id != 0x60)
		return HAL_ERROR;

	ret = BME280_WriteReg(BME280_REG_RESET, BME280_RESET_VALUE);
	if (ret != HAL_OK)
		return ret;
	HAL_Delay(5);

	ret = BME280_ReadCalibration();
	if (ret != HAL_OK)
		return ret;

	ret = BME280_WriteReg(BME280_REG_CTRL_HUM, 0x01);
	if (ret != HAL_OK)
		return ret;

	ret = BME280_WriteReg(BME280_REG_CONFIG, 0xA8);
	if (ret != HAL_OK)
		return ret;

	ret = BME280_WriteReg(BME280_REG_CTRL_MEAS, 0x2F);
	if (ret != HAL_OK)
		return ret;

	ret = BME280_WaitForMeasDone();
	return ret;
}

uint8_t BME280_ReadRaw(int32_t *raw_temp, int32_t *raw_press, int32_t *raw_hum) {
	uint8_t data[8];
	HAL_StatusTypeDef ret;

	ret = BME280_WaitForMeasDone();
	if (ret != HAL_OK)
		return ret;

	// Burst read from 0xF7: press[3], temp[3], hum[2]
	ret = BME280_ReadReg(BME280_REG_PRESS_MSB, data, sizeof(data));
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
	var1 = ((((adcT >> 3) - ((int32_t) calib.dig_T1 << 1)))
			* ((int32_t) calib.dig_T2)) >> 11;

	var2 = (((((adcT >> 4) - ((int32_t) calib.dig_T1))
			* ((adcT >> 4) - ((int32_t) calib.dig_T1))) >> 12)
			* ((int32_t) calib.dig_T3)) >> 14;

	t_fine = var1 + var2;

	// Pressure calculation
	var1 = (((int32_t) t_fine) >> 1) - (int32_t) 64000;
	var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t) calib.dig_P6);
	var2 = var2 + ((var1 * ((int32_t) calib.dig_P5)) << 1);
	var2 = (var2 >> 2) + (((int32_t) calib.dig_P4) << 16);
	var1 = (((((int32_t) calib.dig_P3) * (((var1 >> 2) * (var1 >> 2)) >> 13))
			>> 3) + ((((int32_t) calib.dig_P2) * var1) >> 1)) >> 18;
	var1 = ((32768 + var1) * ((int32_t) calib.dig_P1)) >> 15;

	if (var1 == 0)
		return 0; // avoid division by zero

	p = (((uint32_t) 1048576 - (uint32_t) adcP) - (var2 >> 12)) * 3125;

	if (p < 0x80000000)
		p = (p << 1) / (uint32_t) var1;
	else
		p = (p / (uint32_t) var1) * 2;

	var1 = (((int32_t) calib.dig_P9) * (int32_t) (((p >> 3) * (p >> 3)) >> 13))
			>> 12;

	var2 = (((int32_t) (p >> 2)) * ((int32_t) calib.dig_P8)) >> 13;

	p = (uint32_t) ((int32_t) p + ((var1 + var2 + calib.dig_P7) >> 4));

	return (int32_t) p; // Return Pa
}

// =================================== MPU6050 ==================================
static uint8_t MPU6050_WriteReg(uint8_t reg, uint8_t value) {
	return HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
			&value, 1, 50);
}

static uint8_t MPU6050_ReadReg(uint8_t reg, uint8_t *buf) {
	return HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
			buf, 1, 50);
}

static uint8_t MPU6050_ReadMulti(uint8_t reg, uint8_t *buf, uint16_t len) {
	return HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
			buf, len, 50);
}

bool MPU6050_Init(MPU6050_AccelFs_t accelFs, MPU6050_GyroFs_t gyroFs) {
	sAccelFs = accelFs;
	sGyroFs = gyroFs;

	uint8_t who;
	if (MPU6050_ReadReg(MPU6050_REG_WHO_AM_I, &who) != HAL_OK) {
		return false;
	}
	if (who != MPU6050_WHO_AM_I_VALUE) {
		return false;
	}

	// exit sleep, use pll
	if (MPU6050_WriteReg(MPU6050_REG_PWR_MGMT_1, 0x01) != HAL_OK) {
		return false;
	}

	// sample rate
	if (MPU6050_WriteReg(MPU6050_REG_SMPLRT_DIV, 9) != HAL_OK) {
		return false;
	}

	// DLPF
	if (MPU6050_WriteReg(MPU6050_REG_CONFIG, 0x03) != HAL_OK) {
		return false;
	}

	// full scale gyro
	if (MPU6050_WriteReg(MPU6050_REG_GYRO_CONFIG, (uint8_t) (gyroFs << 3))
			!= HAL_OK) {
		return false;
	}

	// full scale accel
	if (MPU6050_WriteReg(MPU6050_REG_ACCEL_CONFIG, (uint8_t) (accelFs << 3))
			!= HAL_OK) {
		return false;
	}

	return true;
}

bool MPU6050_ReadRaw(MPU6050_Raw_t *out) {
	uint8_t buf[14];

	if (MPU6050_ReadMulti(MPU6050_REG_ACCEL_XOUT_H, buf, sizeof(buf))
			!= HAL_OK) {
		return false;
	}

	out->ax = (int16_t) ((buf[0] << 8) | buf[1]);
	out->ay = (int16_t) ((buf[2] << 8) | buf[3]);
	out->az = (int16_t) ((buf[4] << 8) | buf[5]);

	out->tempRaw = (int16_t) ((buf[6] << 8) | buf[7]);

	out->gx = (int16_t) ((buf[8] << 8) | buf[9]);
	out->gy = (int16_t) ((buf[10] << 8) | buf[11]);
	out->gz = (int16_t) ((buf[12] << 8) | buf[13]);

	return true;
}

void MPU6050_Convert(const MPU6050_Raw_t *raw, MPU6050_AccelFs_t accelFs,
		MPU6050_GyroFs_t gyroFs, MPU6050_Scaled_t *out) {
	int32_t accelLsbPerG = 16384; // for ±2g
	int32_t gyroLsbPerDps = 131;  // for ±250 dps

	switch (accelFs) {
	case MPU6050_ACCEL_FS_2G:
		accelLsbPerG = 16384;
		break;
	case MPU6050_ACCEL_FS_4G:
		accelLsbPerG = 8192;
		break;
	case MPU6050_ACCEL_FS_8G:
		accelLsbPerG = 4096;
		break;
	case MPU6050_ACCEL_FS_16G:
		accelLsbPerG = 2048;
		break;
	}

	switch (gyroFs) {
	case MPU6050_GYRO_FS_250DPS:
		gyroLsbPerDps = 131;
		break;
	case MPU6050_GYRO_FS_500DPS:
		gyroLsbPerDps = 65;
		break;
	case MPU6050_GYRO_FS_1000DPS:
		gyroLsbPerDps = 32;
		break;
	case MPU6050_GYRO_FS_2000DPS:
		gyroLsbPerDps = 16;
		break;
	}

	// Accel: mg
	out->axMg = (int32_t) raw->ax * 1000 / accelLsbPerG;
	out->ayMg = (int32_t) raw->ay * 1000 / accelLsbPerG;
	out->azMg = (int32_t) raw->az * 1000 / accelLsbPerG;

	// Gyro: mdps
	out->gxMdps = (int32_t) raw->gx * 1000 / gyroLsbPerDps;
	out->gyMdps = (int32_t) raw->gy * 1000 / gyroLsbPerDps;
	out->gzMdps = (int32_t) raw->gz * 1000 / gyroLsbPerDps;

}

