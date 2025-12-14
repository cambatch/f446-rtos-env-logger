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


// =================================== MPU6050 ==================================

#endif /* INC_SENSORS_H_ */
