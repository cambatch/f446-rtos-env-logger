/*
 * mpu6050.h
 *
 *  Created on: Dec 14, 2025
 *      Author: Cameron
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include <stdint.h>
#include <stdbool.h>

#define MPU6050_ADDR (0x68 << 1)

#define MPU6050_REG_WHO_AM_I     0x75
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_SMPLRT_DIV   0x19
#define MPU6050_REG_CONFIG       0x1A
#define MPU6050_REG_GYRO_CONFIG  0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_ACCEL_XOUT_H 0x3B

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


#endif /* INC_MPU6050_H_ */
