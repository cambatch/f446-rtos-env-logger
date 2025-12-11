/*
 * sht31.h
 *
 *  Created on: Dec 10, 2025
 *      Author: Cameron
 *
 */

#ifndef INC_SHT31_H_
#define INC_SHT31_H_

#include <stdbool.h>
#include <stdint.h>

#define SHT31_ADDR (0x44 << 1)
#define SHT31_CMD_SOFT_RESET_HIGH 0x30
#define SHT31_CMD_SOFT_RESET_LOW  0xA2
#define SHT31_MEAS_CMD_LOW 0x06U
#define SHT31_MEAS_CMD_HIGH 0x2CU


bool SHT31_Init(void);

bool SHT31_StartMeasurement(void);
bool SHT31_ReadSensor(uint8_t *buf);
void SHT31_ConvertFromRaw(uint8_t *buf, uint32_t *h, int32_t *t);


#endif /* INC_SHT31_H_ */
