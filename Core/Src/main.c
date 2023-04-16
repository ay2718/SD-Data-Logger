/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <retarget.h>
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
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

volatile int rx_half_cplt_flag = 0;
volatile int rx_cplt_flag = 0;

extern uint8_t uart_rx_buf[UART_RX_BUF_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  RetargetInit(&huart1);

  printf("\nSD Card Demo\n\n");

  HAL_Delay(1000);

//  fres = f_mount(&FatFs, "", 1);
//  if (fres != FR_OK) {
//	  printf("f_mount error (%i)\r\n", fres);
//	  while(1);
//  } else {
//	  printf("Mounted successfully!\n\r");
//  }
//
//  DWORD free_clusters, free_sectors, total_sectors;

//  FATFS* getFreeFs;

//  fres = f_getfree("", &free_clusters, &getFreeFs);
//  if (fres != FR_OK) {
//	  printf("g_getfree error (%i)\n\r", fres);
//	  while(1);
//  } else {
//	  printf("Getfree success!\n\r");
//  }

  //Formula comes from ChaN's documentation
//  total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
//  free_sectors = free_clusters * getFreeFs->csize;

//  printf("SD card stats:\r\n%10lu KiB total drive space.\r\n%10lu KiB available.\r\n", total_sectors / 2, free_sectors / 2);


//  fres = f_open(&fil, "test.txt", FA_READ);
//  if (fres != FR_OK) {
//	  printf("f_open error (%i)\r\n", fres);
//	  while(1);
//  }
//  printf("I was able to open 'test.txt' for reading!\r\n");
//
//  //Read 30 bytes from "test.txt" on the SD card
//  BYTE readBuf[30];
//
//  //We can either use f_read OR f_gets to get data out of files
//  //f_gets is a wrapper on f_read that does some string formatting for us
//  TCHAR* rres = f_gets((TCHAR*)readBuf, 30, &fil);
//  if(rres != 0) {
//	  printf("Read string from 'test.txt' contents: %s\r\n", readBuf);
//  } else {
//	  printf("f_gets error (%i)\r\n", fres);
//  }
//
//  //Be a tidy kiwi - don't forget to close your file!
//  f_close(&fil);
//
//  //Now let's try and write a file "write.txt"
//  fres = f_open(&fil, "write.txt", FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
//  if(fres == FR_OK) {
//	  printf("I was able to open 'write.txt' for writing\r\n");
//  } else {
//	  printf("f_open error (%i)\r\n", fres);
//  }
//
//  //Copy in a string
//  for (int i = 0; i < 30; i++) {
//	  readBuf[i] = getchar();
//  }
////  strncpy((char*)readBuf, "a new file is made!", 19);
//  UINT bytesWrote;
//  fres = f_write(&fil, readBuf, 30, &bytesWrote);
//  if(fres == FR_OK) {
//	  printf("Wrote %i bytes to 'write.txt'!\r\n", bytesWrote);
//  } else {
//	  printf("f_write error (%i)\r\n", fres);
//  }
//
//  //Be a tidy kiwi - don't forget to close your file!
//  f_close(&fil);
//
//  //We're done, so de-mount the drive
//  f_mount(NULL, "", 0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
	  if (HAL_GPIO_ReadPin(CARD_DETECT_GPIO_Port, CARD_DETECT_Pin) == GPIO_PIN_SET) {
		  printf("No card detected!\n");
		  HAL_Delay(1000);
		  continue;
	  }
	  HAL_Delay(1000);
	  MX_FATFS_Init();
	  FRESULT fres = f_mount(&USERFatFS, "", 1);
	  if (fres != FR_OK) {
		  printf("f_mount error (%i)\n", fres);
		  f_mount(NULL, "", 0);
		  MX_FATFS_DeInit();
		  continue;
	  }
	  printf("Mounted Successfully!\n");

	  char strprev[128];
	  char returns[128];
	  char date[128];
	  char hours[10];
	  char minutes[10];
	  char seconds[10];

	  scanf("%127[^\n\r]%127[\n\r ]%127[^\n\r,], %127[^\n\r,.:]:%127[^\n\r,.:]:%127[^\n\r,.:]", strprev, returns, date, hours, minutes, seconds);
	  char fname[300];
	  sprintf(fname, "%s--%s;%s;%s.csv", date, hours, minutes, seconds);

	  printf("File name %s\n", fname);

	  fres = f_open(&USERFile, fname, FA_CREATE_ALWAYS | FA_OPEN_ALWAYS | FA_WRITE);
	  if (fres != FR_OK) {
		  printf("f_open error (%i)\n", fres);
		  f_close(&USERFile);
		  f_mount(NULL, "", 0);
		  MX_FATFS_DeInit();
		  continue;
	  }
	  printf("Opened %s\n", fname);

	  while(fres == FR_OK) {
		  if (rx_cplt_flag && rx_half_cplt_flag) {
			  rx_cplt_flag = 0;
			  rx_half_cplt_flag = 0;
		  }
		  if (rx_half_cplt_flag) {
			  rx_half_cplt_flag = 0;
			  UINT bytesWrote;
			  fres = f_write(&USERFile, uart_rx_buf, (UART_RX_BUF_SIZE >> 1), &bytesWrote);
			  f_sync(&USERFile);

			  FATFS* getFreeFs;
			  DWORD free_clusters, free_sectors, total_sectors;
			  f_getfree("", &free_clusters, &getFreeFs);
			  total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
			  free_sectors = free_clusters * getFreeFs->csize;

			  printf("Wrote %i bytes 1/2 ", bytesWrote);
			  printf("%5lu MiB / %5lu MiB\n", (total_sectors - free_sectors) / 2048, total_sectors / 2048);

		  }
		  if (rx_cplt_flag) {
			  rx_cplt_flag = 0;
			  UINT bytesWrote;
			  fres = f_write(&USERFile, uart_rx_buf+(UART_RX_BUF_SIZE >> 1), (UART_RX_BUF_SIZE >> 1), &bytesWrote);
			  f_sync(&USERFile);

			  FATFS* getFreeFs;
			  DWORD free_clusters, free_sectors, total_sectors;
			  f_getfree("", &free_clusters, &getFreeFs);
			  total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
			  free_sectors = free_clusters * getFreeFs->csize;

			  printf("Wrote %i bytes 2/2 ", bytesWrote);
			  printf("%5lu MiB / %5lu MiB\n", (total_sectors - free_sectors) / 2048, total_sectors / 2048);
		  }
	  }

	  printf("f_write error (%i)\n", fres);
	  f_close(&USERFile);
	  f_mount(NULL, "", 0);
	  MX_FATFS_DeInit();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CARD_DETECT_Pin WRITE_PROTECT_Pin */
  GPIO_InitStruct.Pin = CARD_DETECT_Pin|WRITE_PROTECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		rx_half_cplt_flag = 1;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		rx_cplt_flag = 1;
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
