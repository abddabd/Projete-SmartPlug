/* USER CODE BEGIN Header */
/**
  **************************
  * @file           : main.c
  * @brief          : Main program body
  **************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  **************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ST7789/ST7789/st7789.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define device_tag "modem"
#define ubidots_token "BBUS-iMJmzK3U5LcwLZPmgtta2SA47Jm7g4"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
int i;
uint32_t rawCurrentInput; //Pino A0
double currentV;
double currentVRMS;
double currentVMAX;
double currentVMIN;
double currentOutput = 0;
double currentOutputMilis;
uint32_t rawVoltageInput; //Pino A1
double voltageV;
double voltageVMAX;
double voltageVMIN;
double voltageVRMS;
double voltageOutput = 0;
double powerOutput = 0;
double previousVoltageOutput = 0;
double previousCurrentOutput = 0;
double previousPowerOutput = 0;

int indx = 0;
short onOff = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
double sqrt(double);
int sprintf(char *str, const char *format, ...);
int	snprintf (char *__restrict, size_t, const char *__restrict, ...);
void Communicate(void);
void relay_control(short);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char TxData[200];
char incomingData[250];
char RxData[300];
char volString[50];
char curString[50];
char powString[50];
char temString[100];

char ledString[17];
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
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, onOff);
  relay_control(onOff);
  ST7789_Init();

  ST7789_Fill_Color(WHITE);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 1);
  HAL_Delay(1000);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 0);
  HAL_Delay(12500);

  sprintf(TxData, "ATE0\r\n");
  	    HAL_UART_Transmit(&huart1, (uint8_t*)TxData, strlen(TxData), 1000);
		HAL_Delay(2000);
  sprintf(TxData, "AT+GSMBUSY=1\r\n");
      	HAL_UART_Transmit(&huart1, (uint8_t*)TxData, strlen(TxData), 1000);
      	HAL_Delay(2000);
  sprintf(TxData, "AT+CGATT=1\r\n");
    	HAL_UART_Transmit(&huart1, (uint8_t*)TxData, strlen(TxData), 1000);
    	HAL_Delay(2000);
  sprintf(TxData, "AT+CIPMUX=0\r\n");
  	  	HAL_UART_Transmit(&huart1, (uint8_t*)TxData, strlen(TxData), 1000);
    	HAL_Delay(2000);
  sprintf(TxData, "AT+CSTT=\"zap.vivo.com.br\",\"vivo\",\"vivo\"\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)TxData, strlen(TxData), 1000);
        HAL_Delay(5000);
  sprintf(TxData, "AT+CIICR\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)TxData, strlen(TxData), 1000);
        HAL_Delay(2000);
  sprintf(TxData, "AT+CIFSR\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)TxData, strlen(TxData), 1000);
        HAL_Delay(2000);
  sprintf(TxData, "AT+CIPSHUT\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)TxData, strlen(TxData), 1000);
        HAL_Delay(2000);

  sprintf(TxData, "AT+CIPSTART=\"TCP\",\"industrial.api.ubidots.com\",\"80\"\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)TxData, strlen(TxData), 1000);
        HAL_Delay(10000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  currentVMAX = -999999;
	  currentVMIN = 999999;
	  voltageVMAX = -999999;
	  voltageVMIN = 999999;
	 // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, onOff);
	  relay_control(onOff);

	for (i = 0; i < 400; i++) {

	//Corrente
		HAL_ADC_Start(&hadc1);
		rawCurrentInput = HAL_ADC_GetValue(&hadc1);
		currentV = ((double)rawCurrentInput/4096)*3.3;

		if (currentVMAX < currentV) {
			currentVMAX = currentV;
		}
		if (currentVMIN > currentV) {
			currentVMIN = currentV;
		}

		currentV = ((double)rawCurrentInput/4096)*3.3;

		if (currentVMAX < currentV) {
			currentVMAX = currentV;
		}
		if (currentVMIN > currentV) {
			currentVMIN = currentV;
		}

	//Tensão

		HAL_ADC_Start(&hadc2);
		rawVoltageInput = HAL_ADC_GetValue(&hadc2);
		voltageV = ((double)rawVoltageInput/4096)*3.3;

		if (voltageVMAX < voltageV) {
			voltageVMAX = voltageV;
		}
		if (voltageVMIN > voltageV) {
			voltageVMIN = voltageV;
		}

	}

	currentVRMS = (currentVMAX - currentVMIN) / (2 * sqrt(2));
	voltageVRMS = (voltageVMAX - voltageVMIN) / (2 * sqrt(2));

	currentOutput = previousCurrentOutput + 0.97 * ((currentVRMS * 3.94)-previousCurrentOutput);      //currentVRMS * (4.020);
	voltageOutput = previousVoltageOutput + 0.97 * ((voltageVRMS * 668.420)-previousVoltageOutput);
	powerOutput = previousPowerOutput + 0.97 * ((voltageOutput * currentOutput)-previousPowerOutput);

	previousVoltageOutput = voltageOutput;
	previousCurrentOutput = currentOutput;
	previousPowerOutput = powerOutput;

	currentOutputMilis = currentOutput * 1000;

	snprintf(volString, 50, "%1f", voltageOutput);
	sprintf(temString, "Tensao:%13sV", volString);
	ST7789_WriteString(0, 165, temString, Font_11x18, BLACK, WHITE);

	snprintf(curString, 50, "%1f", currentOutputMilis);
	sprintf(temString, "Corrente:%10smA", curString);
	ST7789_WriteString(0, 111, temString, Font_11x18, BLACK, WHITE);
	ST7789_WriteString(0, 129, "   ", Font_11x18, BLACK, WHITE);

	snprintf(powString, 50, "%1f", powerOutput);
	sprintf(temString, "Potencia:%11sW", powString);
	ST7789_WriteString(0, 57, temString, Font_11x18, BLACK, WHITE);

	Communicate();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
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
  huart1.Init.BaudRate = 4800;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ST7789_CS_Pin|GPIO_PIN_8|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ST7789_DC_Pin|ST7789_RST_Pin|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pins : ST7789_CS_Pin PA11 PA12 */
  GPIO_InitStruct.Pin = ST7789_CS_Pin|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ST7789_DC_Pin ST7789_RST_Pin */
  GPIO_InitStruct.Pin = ST7789_DC_Pin|ST7789_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void relay_control(short state)
{
	if(state==0)
	{
		//turn on
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 0);
	}
	else if(state==1)
	{
		//turn off
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
	}
}


void Communicate() {

	char *token;
	char *lastToken = NULL;
	char *secondLastToken = NULL;

	sprintf(TxData, "AT+CIPSEND\r\n");
	       HAL_UART_Transmit(&huart1, (uint8_t*)TxData, strlen(TxData), 1000);
	       HAL_Delay(700);

	sprintf(TxData, "POST /api/v1.6/devices/"device_tag" HTTP/1.1\r\nHost: industrial.api.ubidots.com\r\nContent-Type: application/json\r\nX-Auth-Token: "ubidots_token"\r\nContent-Length: %d\r\n\r\n{\"voltage\":%s,\"current\":%s,\"power\":%s}\x1A", 32 + strlen(volString) + strlen(curString) + strlen(powString), volString, curString, powString);
	       HAL_UART_Transmit(&huart1, (uint8_t*)TxData, strlen(TxData), 1000);
	       HAL_Delay(3700);

	sprintf(TxData, "AT+CIPSEND\r\n");
	       HAL_UART_Transmit(&huart1, (uint8_t*)TxData, strlen(TxData), 1000);
	       HAL_Delay(700);

	sprintf(TxData, "GET /api/v1.6/devices/"device_tag"/onOff/lv HTTP/1.1\r\nHost: industrial.api.ubidots.com\r\nX-Auth-Token: "ubidots_token"\r\n\r\n\x1A");
	       HAL_UART_Transmit(&huart1, (uint8_t*)TxData, strlen(TxData), 1000);
	       HAL_Delay(100);

	       HAL_UART_Receive(&huart1, (uint8_t*)RxData, 900, 6500);

	       token = strtok(RxData, "\r\n");
	           while (token != NULL) {
	               if (strlen(token) > 0) {
	                   secondLastToken = lastToken;
	                   lastToken = token;
	               }
	               token = strtok(NULL, "\r\n");
	           }
	           onOff = atoi(secondLastToken);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
