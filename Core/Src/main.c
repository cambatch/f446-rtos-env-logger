/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdarg.h>

#include "ili9341.h"
#include "sensors.h"
#include "tsl2591.h"
#include "sht31.h"
#include "bme280.h"

#include "queue.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* Definitions for Debug_Task */
osThreadId_t Debug_TaskHandle;
const osThreadAttr_t Debug_Task_attributes =
		{ .name = "Debug_Task", .stack_size = 512 * 4, .priority =
				(osPriority_t) osPriorityBelowNormal, };
/* Definitions for LCD_Task */
osThreadId_t LCD_TaskHandle;
const osThreadAttr_t LCD_Task_attributes = { .name = "LCD_Task", .stack_size =
		512 * 4, .priority = (osPriority_t) osPriorityNormal, };
/* Definitions for Sensor_Task */
osThreadId_t Sensor_TaskHandle;
const osThreadAttr_t Sensor_Task_attributes = { .name = "Sensor_Task",
		.stack_size = 256 * 4, .priority = (osPriority_t) osPriorityNormal1, };
/* Definitions for Logging_Task */
osThreadId_t Logging_TaskHandle;
const osThreadAttr_t Logging_Task_attributes = { .name = "Logging_Task",
		.stack_size = 300 * 4, .priority = (osPriority_t) osPriorityLow, };
/* Definitions for Spi1Mutex */
osMutexId_t Spi1MutexHandle;
const osMutexAttr_t Spi1Mutex_attributes = { .name = "Spi1Mutex" };
/* USER CODE BEGIN PV */
QueueHandle_t xSensorToLcd;
QueueHandle_t xSensorToLog;

static uint32_t sampleCount = 0;

volatile bool gLoggingEnabled = false;

// data from all sensors
SensorData_t gSensorData;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C2_Init(void);
void StartDebugTask(void *argument);
void StartLcdTask(void *argument);
void StartSensorTask(void *argument);
void StartLogTask(void *argument);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void SPI1_SetSpeedHigh(void) {
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
	HAL_SPI_Init(&hspi1);
}

static void debug_printf(const char *fmt, ...) {
#if defined(USE_UART_DEBUG)
	char buf[256];
	va_list args;
	va_start(args, fmt);
	int len = vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);

	if (len > 0) {
		HAL_UART_Transmit(&huart2, (uint8_t*) buf, len, 50);
	}
#else
	(void)fmt;
#endif
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_I2C1_Init();
	MX_SPI1_Init();
	MX_FATFS_Init();
	MX_I2C2_Init();
	/* USER CODE BEGIN 2 */

	// Try to mount a few times
	for (int i = 0; i < 5; ++i) {
		retUSER = f_mount(&USERFatFS, (TCHAR const*) USERPath, 1);
		if (retUSER == FR_OK) {
			break;
		}
		HAL_Delay(10);
	}

	// Let SPI go full speed.
	SPI1_SetSpeedHigh();

	// Initialize LCD
	ILI9341_Init();
	ILI9341_FillScreen(ILI9341_WHITE);

	// Try to initialize TSL2591
	for (int i = 0; i < 5; ++i) {
		if (TSL2591_Init(TSL2591_INT_NONE))
			break;
		HAL_Delay(200);
	}

	// Initialize BME280
	if (BME280_Init() != HAL_OK) {
		Error_Handler();
	}

	if (!MPU6050_Init(MPU6050_ACCEL_FS_2G, MPU6050_GYRO_FS_250DPS)) {
		Error_Handler();
	}

	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();
	/* Create the mutex(es) */
	/* creation of Spi1Mutex */
	Spi1MutexHandle = osMutexNew(&Spi1Mutex_attributes);

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	xSensorToLcd = xQueueCreate(5, sizeof(SensorData_t));
	if (xSensorToLcd == NULL) {
		Error_Handler();
	}

	xSensorToLog = xQueueCreate(5, sizeof(SensorData_t));
	if (xSensorToLog == NULL) {
		Error_Handler();
	}
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of Debug_Task */
	Debug_TaskHandle = osThreadNew(StartDebugTask, NULL,
			&Debug_Task_attributes);

	/* creation of LCD_Task */
	LCD_TaskHandle = osThreadNew(StartLcdTask, NULL, &LCD_Task_attributes);

	/* creation of Sensor_Task */
	Sensor_TaskHandle = osThreadNew(StartSensorTask, NULL,
			&Sensor_Task_attributes);

	/* creation of Logging_Task */
	Logging_TaskHandle = osThreadNew(StartLogTask, NULL,
			&Logging_Task_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 360;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.ClockSpeed = 400000;
	hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */
#ifdef USE_UART_DEBUG
	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */
#endif
	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, ILI9341_CS_Pin | SD_CS_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, ILI_DC_Pin | ILI_RES_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : BTN_Pin */
	GPIO_InitStruct.Pin = BTN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(BTN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : ILI9341_CS_Pin ILI_DC_Pin ILI_RES_Pin SD_CS_Pin */
	GPIO_InitStruct.Pin = ILI9341_CS_Pin | ILI_DC_Pin | ILI_RES_Pin | SD_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	// Enable button interrupt
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 15, 1);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	// PC13 is button
	if (GPIO_Pin == GPIO_PIN_13) {
		BaseType_t higherPriorityTaskWoken = pdFALSE;

		vTaskNotifyGiveFromISR(Debug_TaskHandle, &higherPriorityTaskWoken);

		portYIELD_FROM_ISR(higherPriorityTaskWoken);
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDebugTask */
/**
 * @brief  Function implementing the Debug_Task thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDebugTask */
void StartDebugTask(void *argument) {
	/* USER CODE BEGIN 5 */
	char buf[130];
	UINT bw;
	/* Infinite loop */
	for (;;) {
		// Wait for a notification from ISR
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		osMutexAcquire(Spi1MutexHandle, portMAX_DELAY);
		if (gLoggingEnabled) {
			// Disable logging and close the file
			int len = snprintf(buf, sizeof(buf), "[LOG END]\r\n");
			f_write(&USERFile, buf, len, &bw);
			f_sync(&USERFile);
			f_close(&USERFile);
			gLoggingEnabled = false;
		} else {
			// Enable logging and open file, disable if unable to open file.
			retUSER = f_open(&USERFile, "log.csv", FA_OPEN_APPEND | FA_WRITE);
			if (retUSER != FR_OK) {
				debug_printf("Failed to open 'log.csv'.\r\n");
				gLoggingEnabled = false;
			} else {
				debug_printf("Logging to file 'log.csv'.\r\n");
				gLoggingEnabled = true;
				int len = snprintf(buf, sizeof(buf), "[LOG BEG]\r\n");
				f_write(&USERFile, buf, len, &bw);
			}
		}
		osMutexRelease(Spi1MutexHandle);

		UBaseType_t lcdMin = uxTaskGetStackHighWaterMark(
				(TaskHandle_t) LCD_TaskHandle);
		UBaseType_t sensorMin = uxTaskGetStackHighWaterMark(
				(TaskHandle_t) Sensor_TaskHandle);
		UBaseType_t logMin = uxTaskGetStackHighWaterMark(
				(TaskHandle_t) Logging_TaskHandle);
		UBaseType_t debugMin = uxTaskGetStackHighWaterMark(
				(TaskHandle_t) Debug_TaskHandle);

		debug_printf("LCD free stack: %lu words\r\n"
				"Sensor free stack: %lu words\r\n"
				"Log free stack: %lu words\r\n"
				"Debug free stack: %lu words\r\n"
				"Logging: %s\r\n\r\n", (unsigned long) lcdMin,
				(unsigned long) sensorMin, (unsigned long) logMin,
				(unsigned long) debugMin, gLoggingEnabled ? "ON" : "OFF");

	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartLcdTask */
/**
 * @brief Function implementing the LCD_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartLcdTask */
void StartLcdTask(void *argument) {
	/* USER CODE BEGIN StartLcdTask */
	SensorData_t sensorData = { };

	/* Infinite loop */
	for (;;) {
		if (xQueueReceive(xSensorToLcd, &sensorData, portMAX_DELAY) == pdPASS) {
			osMutexAcquire(Spi1MutexHandle, portMAX_DELAY);

			char line1[50];
			char line2[32];
			char line3[25];
			char line4[30];
			char line5[40];
			char line6[15];

			// Humidity stored in 0-1000 (0.0-100.0) and time in milli C
			int32_t t = sensorData.temperature;
			int32_t temp_int = t / 1000;
			int32_t temp_frac = (t >= 0) ? (t % 1000) : ((-t) % 1000);
			uint32_t h = sensorData.humidity;

			snprintf(line1, sizeof(line1),
					"T: %ld.%03ld C    H: %lu.%01lu %%RH", temp_int, temp_frac, h / 10, h % 10);

			// Format lux
			if (sensorData.lux < 0) {
				snprintf(line2, sizeof(line2), "Saturated");
			} else {
				snprintf(line2, sizeof(line2), "%ld lx", (long)sensorData.lux);
			}

			// Pressure
			uint32_t p_int = sensorData.pressure / 100;
			uint32_t p_frac = sensorData.pressure % 100;
			snprintf(line3, sizeof(line3), "P: %ld.%01ld hPa", p_int, p_frac);

			// accel
			snprintf(line4, sizeof(line4), "Ax:%4ld Ay:%4ld Az:%4ld",
					sensorData.accelX, sensorData.accelY, sensorData.accelZ);

			// gyro
			int32_t gxDps = sensorData.gyroX / 1000;
			int32_t gyDps = sensorData.gyroY / 1000;
			int32_t gzDps = sensorData.gyroZ / 1000;
			snprintf(line5, sizeof(line5), "Gx:%ld Gy:%ld Gz:%ld", (long) gxDps,
					(long) gyDps, (long) gzDps);

			// Logging status
			snprintf(line6, sizeof(line6), "Logging %s",
					gLoggingEnabled ? "ON" : "OFF");

			ILI9341_FillRectangle(0, 0, ILI9341_WIDTH, 20, ILI9341_WHITE);
			ILI9341_WriteString(2, 2, line1, Font_11x18, ILI9341_BLACK,
			ILI9341_WHITE);

			ILI9341_FillRectangle(0, 22, ILI9341_WIDTH, 20, ILI9341_WHITE);
			ILI9341_WriteString(2, 24, line2, Font_11x18, ILI9341_BLACK,
			ILI9341_WHITE);

			ILI9341_FillRectangle(0, 44, ILI9341_WIDTH, 20, ILI9341_WHITE);
			ILI9341_WriteString(2, 46, line3, Font_11x18, ILI9341_BLACK,
			ILI9341_WHITE);

			ILI9341_FillRectangle(0, 66, ILI9341_WIDTH, 20, ILI9341_WHITE);
			ILI9341_WriteString(2, 68, line4, Font_11x18, ILI9341_BLACK,
			ILI9341_WHITE);

			ILI9341_FillRectangle(0, 88, ILI9341_WIDTH, 20, ILI9341_WHITE);
			ILI9341_WriteString(2, 90, line5, Font_11x18, ILI9341_BLACK,
			ILI9341_WHITE);

			ILI9341_FillRectangle(0, 108, ILI9341_WIDTH, 20, ILI9341_WHITE);
			ILI9341_WriteString(2, 110, line6, Font_11x18, ILI9341_BLACK,
			ILI9341_WHITE);

			osMutexRelease(Spi1MutexHandle);
		}
	}
	/* USER CODE END StartLcdTask */
}

/* USER CODE BEGIN Header_StartSensorTask */
/**
 * @brief Function implementing the Sensor_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void *argument) {
	/* USER CODE BEGIN StartSensorTask */
	SensorData_t sensorData = {};

	const TickType_t taskDelay = pdMS_TO_TICKS(1000);

	// SHT31
	const TickType_t measureDelay = pdMS_TO_TICKS(20);
	uint8_t data[6] = { };

	// TSL2591
	uint16_t chs[2] = { };

	int32_t raw_press;
	int32_t raw_temp;
	int32_t raw_humid;

	MPU6050_Raw_t imuRaw;
	MPU6050_Scaled_t imuScaled;

	/* Infinite loop */
	for (;;) {
		sensorData.timestamp = xTaskGetTickCount();

		SHT31_StartMeasurement();

		vTaskDelay(measureDelay);

		SHT31_ReadSensor(data);
		SHT31_ConvertFromRaw(data, &sensorData.humidity,
				&sensorData.temperature);

		TSL2591_ReadChannels(&chs[0], &chs[1]);
		sensorData.lux = TSL2591_CalcIntLux(chs[0], chs[1]);

		BME280_ReadRaw(&raw_temp, &raw_press, &raw_humid);
		sensorData.pressure = BME280_CompensatePressure(raw_press, raw_temp);

		// MPU read
		if (MPU6050_ReadRaw(&imuRaw)) {
			MPU6050_Convert(&imuRaw, MPU6050_ACCEL_FS_2G,
					MPU6050_GYRO_FS_250DPS, &imuScaled);

			sensorData.accelX = imuScaled.axMg;
			sensorData.accelY = imuScaled.ayMg;
			sensorData.accelZ = imuScaled.azMg;
			sensorData.gyroX = imuScaled.gxMdps;
			sensorData.gyroY = imuScaled.gyMdps;
			sensorData.gyroZ = imuScaled.gzMdps;
		}

		xQueueSend(xSensorToLcd, &sensorData, 0);

		// Log every 5 seconds
		if (gLoggingEnabled && (sampleCount % 2) == 0) {
			xQueueSendToBack(xSensorToLog, &sensorData, 0);
		}

		++sampleCount;

		vTaskDelay(taskDelay);
	}
	/* USER CODE END StartSensorTask */
}

/* USER CODE BEGIN Header_StartLogTask */
/**
 * @brief Function implementing the Logging_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartLogTask */
void StartLogTask(void *argument) {
	/* USER CODE BEGIN StartLogTask */
	SensorData_t sData;
	UINT bw;
	uint32_t count = 0;

	/* Infinite loop */
	for (;;) {
		if (xQueueReceive(xSensorToLog, &sData, portMAX_DELAY) == pdPASS) {
			char line[64];
			// timestamp,temperature,humidity,pressure(Pa),lux,accelx,accely,accelz,gyrox,gyroy,gyroz
			int len = snprintf(line, sizeof(line),
					"%lu,%ld,%lu,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld\r\n",
					(unsigned long) sData.timestamp,
					(long int) sData.temperature,
					(unsigned long) sData.humidity, (long int) sData.pressure,
					(long int) sData.lux, (long int) sData.accelX,
					(long int) sData.accelY, (long int) sData.accelZ,
					(long int) sData.gyroX, (long int) sData.gyroY,
					(long int) sData.gyroZ);

			osMutexAcquire(Spi1MutexHandle, portMAX_DELAY);

			f_write(&USERFile, line, len, &bw);

			// sync every 5 writes
			if (count % 5 == 0) {
				f_sync(&USERFile);
			}

			++count;

			osMutexRelease(Spi1MutexHandle);
		}
	}
	/* USER CODE END StartLogTask */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM6) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
