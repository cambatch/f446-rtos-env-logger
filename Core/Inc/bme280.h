/*
 * bme280.h
 *
 *  Created on: Dec 14, 2025
 *      Author: Cameron
 */

#ifndef INC_BME280_H_
#define INC_BME280_H_

#include <stdint.h>


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
	uint16_t digT1;
	int16_t digT2;
	int16_t digT3;

	/* Pressure calibration */
	uint16_t digP1;
	int16_t digP2;
	int16_t digP3;
	int16_t digP4;
	int16_t digP5;
	int16_t digP6;
	int16_t digP7;
	int16_t digP8;
	int16_t digP9;

	/* Humidity calibration */
	uint8_t digH1;
	int16_t digH2;
	uint8_t digH3;
	int16_t digH4;
	int16_t digH5;
	int8_t digH6;
} BME280_CalibData_t;

uint8_t BME280_Init(void);
uint8_t BME280_ReadRaw(int32_t *raw_temp, int32_t *raw_press, int32_t *raw_hum);
int32_t BME280_CompensatePressure(int32_t adcP, int32_t adcT);

#endif /* INC_BME280_H_ */
