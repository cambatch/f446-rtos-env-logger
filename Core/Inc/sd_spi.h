/*
 * sd_spi.h
 *
 *  Created on: Dec 4, 2025
 *      Author: Cameron
 */

#ifndef INC_SD_SPI_H_
#define INC_SD_SPI_H_

#include "stdint.h"
#include "diskio.h"   // FatFS DSTATUS, DRESULT, etc.

/* Card type flags (subset) */
#define CT_MMC     0x01
#define CT_SD1     0x02
#define CT_SD2     0x04
#define CT_SDC     (CT_SD1 | CT_SD2)
#define CT_BLOCK   0x08   /* Block addressing (SDHC/SDXC) */

DSTATUS SD_SPI_Initialize(void);
DSTATUS SD_SPI_Status(void);
DRESULT SD_SPI_Read(BYTE *buff, DWORD sector, UINT count);
DRESULT SD_SPI_Write(const BYTE *buff, DWORD sector, UINT count);
DRESULT SD_SPI_Ioctl(BYTE cmd, void *buff);

#endif /* INC_SD_SPI_H_ */
