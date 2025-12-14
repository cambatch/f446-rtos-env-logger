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

#include "tsl2591.h"

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

typedef struct {
	uint32_t timestamp;
	int32_t temperature; // temp_x10
	uint32_t humidity;	 // humidity_x10

	TSL2591_SensorData_t tslData;
	int32_t lux;		 // lux

	int32_t pressure;	 // Pa

	int32_t accelX;
	int32_t accelY;
	int32_t accelZ;

	int32_t gyroX;
	int32_t gyroY;
	int32_t gyroZ;
} SensorData_t;


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
} BME280_CalibData_t;

uint8_t BME280_Init(void);
uint8_t BME280_ReadRaw(int32_t *raw_temp, int32_t *raw_press, int32_t *raw_hum);
int32_t BME280_CompensatePressure(int32_t adcP, int32_t adcT);

// =================================== MPU6050 ==================================
#define MPU6050_ADDR (0x68 << 1)

// Register map (only what we need)
#define MPU6050_REG_WHO_AM_I     0x75
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_SMPLRT_DIV   0x19
#define MPU6050_REG_CONFIG       0x1A
#define MPU6050_REG_GYRO_CONFIG  0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_ACCEL_XOUT_H 0x3B

// WHO_AM_I expected value
#define MPU6050_WHO_AM_I_VALUE   0x68

typedef enum {
	MPU6050_ACCEL_FS_2G = 0,
	MPU6050_ACCEL_FS_4G = 1,
	MPU6050_ACCEL_FS_8G = 2,
	MPU6050_ACCEL_FS_16G = 3,
} MPU6050_AccelFs_t;

typedef enum {
	MPU6050_GYRO_FS_250DPS = 0,
	MPU6050_GYRO_FS_500DPS = 1,
	MPU6050_GYRO_FS_1000DPS = 2,
	MPU6050_GYRO_FS_2000DPS = 3,
} MPU6050_GyroFs_t;

typedef struct {
	int16_t ax;
	int16_t ay;
	int16_t az;
	int16_t gx;
	int16_t gy;
	int16_t gz;
	int16_t tempRaw;
} MPU6050_Raw_t;

typedef struct {
	int32_t axMg;     // accel in mg
	int32_t ayMg;
	int32_t azMg;
	int32_t gxMdps;   // gyro in mdps
	int32_t gyMdps;
	int32_t gzMdps;
} MPU6050_Scaled_t;

bool MPU6050_Init(MPU6050_AccelFs_t accelFs, MPU6050_GyroFs_t gyroFs);
bool MPU6050_ReadRaw(MPU6050_Raw_t *out);
void MPU6050_Convert(const MPU6050_Raw_t *raw, MPU6050_AccelFs_t accelFs,
		MPU6050_GyroFs_t gyroFs, MPU6050_Scaled_t *out);

#endif /* INC_SENSORS_H_ */
