# f446-rtos-env-logger

A real-time embedded application running on an STM32 microcontroller.  
It reads temperature, humidity, and ambient light uing digital sensors,  
displays live data on an ILI9341 TFT LCD, and logs samples to an SD card  
using SPI-based FatFS implementation.  
FreeRTOS manages task scheduling, message queues, and SPI resource sharing.

## Features

- Real-time sensor acquisition
    - SHT31 (temperature + humidity over I²C)
    - TSL2591 (ambient light lux calculation, medium gain, 100 ms integration)
- User interface
    - ILI9341 LCD over SPI
    - Live display of:
        - Temperature (°C)
        - Relative Humidity (%RH)
        - Illuminance (lux)
        - Logging status
- Data logging
    - SD card logging via SPI
    - FAT32 filesystem (FatFS)
    - CSV output

## Hardware

- **MCU**: STM32F4 series
- **Display**: ILI9341 TFT LCD (SPI mode)
- **Sensors**:
    - SHT31 (I²C)
    - TSL2591 (I²C)
- **Storage**: microSD card (SPI mode)

## Build Instructions

### Requirements

- STM32CubeIDE or GCC toolchain
- STM32 HAL drivers
- FreeRTOS (CMSIS-OS v2 backend)
- FatFS

### Build

1. Open Project in STM32CubeIDE
2. Build
3. Flash via ST-Link

## Usage

1. Power board
2. Live sensor data is shown on LCD
3. Press the button to toggle logging
4. Remove SD card and open log.csv
- **No SD card present**
    - Mount attempts fail quickly through timeouts
    - System still runs normally without logging
