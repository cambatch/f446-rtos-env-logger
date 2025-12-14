/*
 * tsl2591.c
 *
 *  Created on: Dec 2, 2025
 *      Author: Cameron
 */

#include "sensors.h"

static MPU6050_AccelFs_t sAccelFs;
static MPU6050_GyroFs_t sGyroFs;

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

