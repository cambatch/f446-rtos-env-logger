/*
 * tsl2591.h
 *
 *  Created on: Dec 2, 2025
 *      Author: Cameron
 */

#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_

#include "tsl2591.h"
#include "sht31.h"
#include "bme280.h"
#include "mpu6050.h"



typedef struct {
	bool ok;
	uint16_t ch1;
	uint16_t ch2;
	int32_t lux;
} TSL2591_SensorData_t;

typedef struct {
	bool ok;
	uint32_t humidity;   // x1000
	int32_t temperature; // x10
} SHT31_SensorData_t;

typedef struct {
	bool ok;
	int32_t pressure;
	int32_t temperature;
	int32_t humidity;
} BME280_SensorData_t;

typedef struct {
	bool ok;
	int32_t accelX;
	int32_t accelY;
	int32_t accelZ;
	int32_t gyroX;
	int32_t gyroY;
	int32_t gyroZ;
} MPU6050_SensorData_t;

typedef struct {
	uint32_t timestamp;

	SHT31_SensorData_t shtData;

	TSL2591_SensorData_t tslData;

	BME280_SensorData_t bmeData;

	MPU6050_SensorData_t mpuData;

} SensorData_t;

typedef enum {
	SENSOR_STATE_ERROR = 0,
	SENSOR_STATE_OK
} SensorState_t;

#endif /* INC_SENSORS_H_ */
