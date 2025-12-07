/*
 * sd_spi.c
 *
 *  Created on: Dec 4, 2025
 *      Author: Cameron
 */

#include "sd_spi.h"
#include "stm32f4xx_hal.h"
#include "main.h"

#define SPI_TIMEOUT_MS 10


extern SPI_HandleTypeDef hspi1;

#define SD_CS_LOW()   HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET)
#define SD_CS_HIGH()  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET)

/* Timeouts (tweak if needed) */
#define SD_CMD_TIMEOUT     1000u
#define SD_TOKEN_TIMEOUT   2000u

/* R1 response bits */
#define R1_IDLE_STATE      0x01
#define R1_ILLEGAL_CMD     0x04

/* Data token for single block read */
#define TOKEN_START_BLOCK  0xFE

/* Card type (global) */
static BYTE CardType = 0;

/* ------------------- Low-level SPI helpers ------------------- */

static uint8_t spi_txrx(uint8_t data)
{
    uint8_t rx;
    HAL_SPI_TransmitReceive(&hspi1, &data, &rx, 1, SPI_TIMEOUT_MS);
    return rx;
}

static void spi_tx(uint8_t data)
{
    uint8_t dummy;
    HAL_SPI_TransmitReceive(&hspi1, &data, &dummy, 1, SPI_TIMEOUT_MS);
}

static uint8_t spi_rx(void)
{
    return spi_txrx(0xFF);
}

static void send_dummy_clocks(uint8_t count)
{
    SD_CS_HIGH();
    for (uint8_t i = 0; i < count; i++)
        spi_tx(0xFF);
}

/* Wait until card returns 0xFF (not busy), or timeout */
static int wait_not_busy(void)
{
    uint16_t t = SD_TOKEN_TIMEOUT;
    uint8_t b;

    do {
        b = spi_rx();
        if (b == 0xFF) return 1;
    } while (--t);

    return 0;   /* timeout */
}

/* ------------------- SD Command helpers ------------------- */

/* Send a command (CMDxx) and get R1 response */
static uint8_t send_cmd(uint8_t cmd, uint32_t arg)
{
    uint8_t r1;
    uint8_t crc = 0x01;  // default dummy CRC

    if (cmd == 0) crc = 0x95;  // CMD0 correct CRC
    if (cmd == 8) crc = 0x87;  // CMD8 correct CRC

    SD_CS_LOW();
    spi_tx(0xFF);  // Ncr (>=1)

    /* Command packet */
    spi_tx(0x40 | cmd);
    spi_tx((uint8_t)(arg >> 24));
    spi_tx((uint8_t)(arg >> 16));
    spi_tx((uint8_t)(arg >> 8));
    spi_tx((uint8_t)(arg));
    spi_tx(crc);

    /* Wait for response (R1), max 8 bytes */
    for (uint8_t i = 0; i < 8; i++) {
        r1 = spi_rx();
        if ((r1 & 0x80) == 0) break;
    }

    return r1;
}

/* Send ACMD (application specific) = CMD55 + CMDxx */
static uint8_t send_acmd(uint8_t acmd, uint32_t arg)
{
    uint8_t r1;

    r1 = send_cmd(55, 0);
    SD_CS_HIGH();
    spi_rx();  // extra clock

    if (r1 > 1) return r1;
    r1 = send_cmd(acmd, arg);
    return r1;
}

/* ------------------- Public functions for FatFS ------------------- */

DSTATUS SD_SPI_Initialize(void)
{
    uint8_t r1;
    uint8_t ocr[4];
    uint16_t t;

    if (CardType != 0) {
        return 0;  // already initialized
    }

    /* 1. Supply minimum 74 clock cycles with CS high */
    send_dummy_clocks(10);

    /* 2. CMD0: go to idle state */
    t = SD_CMD_TIMEOUT;
    do {
        r1 = send_cmd(0, 0);
        SD_CS_HIGH();
        spi_rx();  // extra
    } while ((r1 != R1_IDLE_STATE) && --t);

    if (!t) {
        SD_CS_HIGH();
        return STA_NOINIT;
    }

    /* 3. CMD8 to check SD v2 */
    r1 = send_cmd(8, 0x1AA);
    for (int i = 0; i < 4; i++) {
        ocr[i] = spi_rx();
    }
    SD_CS_HIGH();
    spi_rx();

    if (r1 == R1_IDLE_STATE && ocr[2] == 0x01 && ocr[3] == 0xAA) {
        /* SD v2 (supports CMD8) */

        /* 4. ACMD41 with HCS bit until it leaves idle */
        t = SD_CMD_TIMEOUT;
        do {
            r1 = send_acmd(41, 0x40000000UL);  // HCS bit
            SD_CS_HIGH();
            spi_rx();
        } while (r1 != 0 && --t);

        if (!t) {
            CardType = 0;
            return STA_NOINIT;
        }

        /* 5. CMD58 to read OCR and check CCS (SDHC/SDXC) */
        r1 = send_cmd(58, 0);
        for (int i = 0; i < 4; i++) {
            ocr[i] = spi_rx();
        }
        SD_CS_HIGH();
        spi_rx();

        if (r1 == 0 && (ocr[0] & 0x40)) {
            /* SDHC / SDXC: block addressing */
            CardType = CT_SD2 | CT_BLOCK;
        } else {
            /* SDSC v2: byte addressing */
            CardType = CT_SD2;
        }
    } else {
        /* SD v1 or MMC: not fully handled here (simple example) */
        CardType = 0;
        return STA_NOINIT;
    }

    /* Optionally: increase SPI speed here by reconfiguring hspi1.Init.BaudRatePrescaler... */

    return 0;   /* STA_OK */
}

DSTATUS SD_SPI_Status(void)
{
    return (CardType ? 0 : STA_NOINIT);
}

/* Read 'count' sectors (512 bytes each) starting at 'sector' */
DRESULT SD_SPI_Read(BYTE *buff, DWORD sector, UINT count)
{
    uint8_t token;
    uint16_t t;

    if (!CardType) return RES_NOTRDY;
    if (!count)    return RES_PARERR;

    /* Convert sector to byte address if needed */
    if (!(CardType & CT_BLOCK)) {
        sector *= 512;
    }

    while (count--) {
        if (send_cmd(17, (uint32_t)sector) != 0) {
            SD_CS_HIGH();
            spi_rx();
            return RES_ERROR;
        }

        /* Wait for data token 0xFE */
        t = SD_TOKEN_TIMEOUT;
        do {
            token = spi_rx();
        } while ((token == 0xFF) && --t);

        if (token != TOKEN_START_BLOCK) {
            SD_CS_HIGH();
            spi_rx();
            return RES_ERROR;
        }

        /* Read 512 bytes */
        for (uint16_t i = 0; i < 512; i++) {
            buff[i] = spi_rx();
        }

        /* Discard CRC */
        spi_rx();
        spi_rx();

        SD_CS_HIGH();
        spi_rx();

        buff += 512;
        sector++;
    }

    return RES_OK;
}

/* Write 'count' sectors starting at 'sector' */
DRESULT SD_SPI_Write(const BYTE *buff, DWORD sector, UINT count)
{
#if _USE_WRITE == 0
    return RES_WRPRT;
#else
    uint8_t resp;

    if (!CardType) return RES_NOTRDY;
    if (!count)    return RES_PARERR;

    /* Convert sector to byte address if needed */
    if (!(CardType & CT_BLOCK)) {
        sector *= 512;
    }

    while (count--) {
        if (send_cmd(24, (uint32_t)sector) != 0) {   /* CMD24: write single block */
            SD_CS_HIGH();
            spi_rx();
            return RES_ERROR;
        }

        /* Send one byte gap then data token */
        spi_tx(0xFF);
        spi_tx(TOKEN_START_BLOCK);

        /* Send 512 bytes data */
        for (uint16_t i = 0; i < 512; i++) {
            spi_tx(buff[i]);
        }

        /* Send dummy CRC */
        spi_tx(0xFF);
        spi_tx(0xFF);

        /* Get data response */
        resp = spi_rx();
        if ((resp & 0x1F) != 0x05) {  /* 0bxxx00101 = accepted */
            SD_CS_HIGH();
            spi_rx();
            return RES_ERROR;
        }

        /* Wait until card not busy */
        if (!wait_not_busy()) {
            SD_CS_HIGH();
            spi_rx();
            return RES_ERROR;
        }

        SD_CS_HIGH();
        spi_rx();

        buff += 512;
        sector++;
    }

    return RES_OK;
#endif
}

DRESULT SD_SPI_Ioctl(BYTE cmd, void *buff)
{
    DRESULT res = RES_ERROR;

    if (!CardType) return RES_NOTRDY;

    switch (cmd) {
    case CTRL_SYNC:
        /* Ensure no pending write */
        SD_CS_LOW();
        if (wait_not_busy()) res = RES_OK;
        SD_CS_HIGH();
        spi_rx();
        break;

    case GET_SECTOR_SIZE:
        *(WORD *)buff = 512;
        res = RES_OK;
        break;

    case GET_BLOCK_SIZE:
        *(DWORD *)buff = 1;  /* erase block size in sectors (dummy) */
        res = RES_OK;
        break;

    case GET_SECTOR_COUNT:
        /* Optional: parse CSD via CMD9 to get real size.
           For now, return 0 to indicate unknown. */
        *(DWORD *)buff = 0;
        res = RES_OK;
        break;

    default:
        res = RES_PARERR;
        break;
    }

    return res;
}

