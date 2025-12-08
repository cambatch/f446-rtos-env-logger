# f446-rtos-env-logger

A real-time embedded application running on an STM32 microcontroller.  
It collects environmental and motion data from multiple digital sensors, displays live telemetry on an ILI9341 TFT LCD, and logs samples to an SD card using an SPI-based FatFS implementation.  
FreeRTOS manages task scheduling, inter-task communication, and protected SPI access.


## Features

### Sensor Acquisition

- **SHT31** – Temperature & humidity (I²C)
- **TSL2591** – Ambient light (I²C), lux calculation
- **BME280** – Barometric pressure (I²C)
- **MPU6050** – 3-axis accelerometer & gyroscope (I²C)

### Display (ILI9341 over SPI)

Live telemetry:
- Temperature (°C)
- Relative Humidity (%RH)
- Illuminance (lux)
- Pressure (hPa)
- Acceleration (mg)
- Angular velocity (°/s)
- Logging state

### Data Logging

- microSD card (SPI mode)
- FAT32 filesystem (FatFS)
- CSV output format:
    - timestamp, T, RH, lux, pressure, ax, ay, az, gx, gy, gz
- Logging toggle via button interrupt
- Short timeout & retry if SD card is missing

### RTOS Architecture (FreeRTOS)

- **Tasks**
    - Sensor task (1 Hz)
    - LCD UI task
    - Logging task
    - Debug/UI task
- **Queues**
    - Sensor → LCD
    - Sensor → Logger
- **Synchronization**
    - Mutex-protected SPI
    - ISR-driven notifications
- **Safety**
    - No floating-point math in drivers


## Hardware

- **MCU**: STM32F446RE (Nucleo-64)
- **Display**: ILI9341 TFT LCD (SPI)
- **Sensors**:
    - SHT31, TSL2591, BME280, MPU6050 (I²C)
- **Storage**: microSD card (SPI mode)

## Build Instructions

### Requirements

- STM32CubeIDE (or arm-none-eabi GCC toolchain)
- STM32 HAL
- FreeRTOS (CMSIS-OS v2)
- FatFS

### Build

1. Clone project
2. Open with STM32CubeIDE
3. Build
4. Flash using ST-Link


## Usage

1. Power the board
2. Live data appears on LCD
3. Press user button to start/stop logging
4. Remove SD card and open `log.csv`

Notes:
- If no card is present: initialization retries briefly, then runs normally without logging.
