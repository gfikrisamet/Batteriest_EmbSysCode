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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
usart_t usart;
batValue_t batValue;
relayStatus_e relayStatus = relayStatus_Idle;

bool dch_m2 = true;
bool adc_buf = false;
bool falan = false;

uint8_t count = 0;

float lts_Vsamp = 0.07412;
float Vrefint = 3.142, Crefint = 2.551;

uint16_t adc_voltage_value[NOS], adc_current_value[NOS], adc_vrefint_value[NOS],
		adc_crefint_value[NOS];
uint16_t adc_temp, adc_voltage, adc_current;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ADC CHANNEL CONFIGURATIONS (MULTI-CHANNEL WITHOUT DMA) //
void ADC_Select_Current() {
	ADC_ChannelConfTypeDef sConfig = { 0 };

	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;

	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
}
void ADC_Select_Temp() {
	ADC_ChannelConfTypeDef sConfig = { 0 };

	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;

	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
}

void ADC_Select_Voltage() {
	ADC_ChannelConfTypeDef sConfig = { 0 };

	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;

	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
}
void ADC_Select_Vrefint() {
	ADC_ChannelConfTypeDef sConfig = { 0 };

	sConfig.Channel = ADC_CHANNEL_VREFINT;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;

	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
}
//END CODE OF ADC CHANNEL CONFIGURATIONS (MULTI-CHANNEL WITHOUT DMA) //

// READING AND PROCESSING ADC VALUES WITH MOVING AVERAGE //
uint16_t Get_ADC_Value() {
	uint16_t adc_value = 0;

	HAL_ADC_Start(&hadc1);
	if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK) {
		adc_value = HAL_ADC_GetValue(&hadc1);
	}
	HAL_ADC_Stop(&hadc1);
	return adc_value;
}

float Get_ADC_MA_Vref(uint16_t *adc_vrefint_buffer) {
	uint64_t sum_adc_value = 0;
	uint8_t i = 0;
	for (; i < NOS; i++) {
		sum_adc_value = sum_adc_value + adc_vrefint_buffer[i];
	}
	return sum_adc_value / NOS;
}

float Get_ADC_Voltage(float raw_voltage) {
	return (raw_voltage / 4095.0) * Vrefint * 2.0 *1.03;
}

float Get_ADC_Vrefint(float raw_vrefint) {
	return (raw_vrefint / *VREFIN_CAL) * Vrefint;
}

float Get_ADC_Current(float raw_current) {
	return (((raw_current / 4095.0) * Vrefint * 2.0) - Crefint) / (lts_Vsamp);
}

double Get_ADC_Temp(uint16_t adc_value) {
	double temp;
	temp = log(((40950000 / adc_value) - 10000));
	temp = 1
			/ (0.001129148
					+ (0.000234125 + (0.0000000876741 * temp * temp)) * temp);
	temp = temp - 273.15;
	return temp;
}

//float Get_ADC_Moving_Average(uint16_t *adc_value_buffer) {
//	float sum_adc_value = 0;
//	uint8_t i = 0;
//	for (; i < NOS; i++) {
//		sum_adc_value = sum_adc_value + adc_value_buffer[i];
//
//	}
//	sum_adc_value = sum_adc_value / NOS;
//	return (sum_adc_value / 4095.0) * Vrefint * 2.0;
//}
// END CODE OF READING AND PROCESSING ADC VALUES WITH MOVING AVERAGE //

void Calibration_ADC() {
	ADC_Select_Vrefint();
	for (int var = 0; var < NOS; ++var) {
		adc_vrefint_value[var] = Get_ADC_Vrefint(Get_ADC_Value());
	}
	Vrefint = Get_ADC_MA_Vref(adc_vrefint_value);
	ADC_Select_Current();
	for (int var = 0; var < NOS; ++var) {
		adc_crefint_value[var] = Get_ADC_Value();
	}
	Crefint = Get_ADC_Moving_Average(adc_crefint_value);
}

// RELAY (GPIO CHANNEL) CONFIGURATIONS //
void Bat_Relay_Cmd() {
	switch (relayStatus) {
	case relayStatus_Idle:
		HAL_GPIO_WritePin(RELAY_1_GPIO_Port, RELAY_1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RELAY_2_GPIO_Port, RELAY_2_Pin, GPIO_PIN_SET);
		HAL_Delay(100);
		dch_m2 = false;
		falan = true;
		break;
	case relayStatus_Charge:
		HAL_GPIO_WritePin(RELAY_1_GPIO_Port, RELAY_1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RELAY_2_GPIO_Port, RELAY_2_Pin, GPIO_PIN_SET);
		HAL_Delay(100);
		dch_m2 = false;
		break;
	case relayStatus_Discharge_M1:
		HAL_GPIO_WritePin(RELAY_1_GPIO_Port, RELAY_1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RELAY_2_GPIO_Port, RELAY_2_Pin, GPIO_PIN_RESET);
		HAL_Delay(100);
		dch_m2 = false;
		break;
	case relayStatus_Discharge_M2:
		HAL_GPIO_WritePin(RELAY_1_GPIO_Port, RELAY_1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RELAY_2_GPIO_Port, RELAY_2_Pin, GPIO_PIN_RESET);
		HAL_Delay(100);
		dch_m2 = true;
		break;
	}
}
// END OF RELAY (GPIO CHANNEL) CONFIGURATIONS //

// UART RECEIVE DATA CONFIGURATIONS //
void UART_Cmd(char *cmd) {
	if (!strcmp(cmd, IDLE)) {
		relayStatusChange(relayStatus_Idle);
	} else if (!strcmp(cmd, CHARGE)) {
		relayStatusChange(relayStatus_Charge);
	} else if (!strcmp(cmd, DISCHARGE_M1)) {
		relayStatusChange(relayStatus_Discharge_M1);
	} else if (!strcmp(cmd, DISCHARGE_M2)) {
		relayStatusChange(relayStatus_Discharge_M2);
	}
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	if (huart->Instance == USART1) {
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t*) usart.rx_data, 20);
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
		for (uint8_t i = Size; i < 20; i++) {
			usart.rx_data[i] = 0;
		}
		usart.received = true;
	}
}
// END CODE OF UART RECEIVE DATA CONFIGURATIONS //

// UART TRANSMIT DATA CONFIGURATIONS //
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == htim1.Instance) {
		if (dch_m2 == false) {
			usart.transtick++;
			if (usart.transtick >= 1000) {
				HAL_UART_Transmit(&huart1, usart.tx_data, sizeof(usart.tx_data),
						1);
				usart.transtick = 0;
			}
		} else if (dch_m2 == true) {
			usart.transtick++;
			if (usart.transtick >= 1) {
				HAL_UART_Transmit(&huart1, usart.tx_data, sizeof(usart.tx_data),
						1);
				usart.transtick = 0;
			}
		}
	}
}
// END CODE OF UART TRANSMIT DATA CONFIGURATIONS //
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
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	Bat_Relay_Cmd(relayStatus);
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t*) usart.rx_data, 20);
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

		switch (dch_m2) {
		case true:
			ADC_Select_Temp();
			batValue.temp = Get_ADC_Temp(Get_ADC_Value());
			ADC_Select_Voltage();
			batValue.voltage = Get_ADC_Voltage(Get_ADC_Value());
			ADC_Select_Current();
			batValue.current = Get_ADC_Current(Get_ADC_Value());
			sprintf((char*) usart.tx_data, "%d,%1.3f,%1.3f,%1.3f\n",
					relayStatus, batValue.voltage, batValue.current,
					batValue.temp);
			break;
		case false:
			ADC_Select_Temp();
			batValue.temp = Get_ADC_Temp(Get_ADC_Value());
			ADC_Select_Voltage();
			adc_voltage_value[count] = Get_ADC_Value();
			ADC_Select_Current();
			adc_current_value[count] = Get_ADC_Value();
			count++;
			if (count == NOS) {
				count = 0;
				adc_buf = true;
			}
			if (adc_buf == true) {
				batValue.voltage = Get_ADC_Voltage(Get_ADC_MA_Vref(adc_voltage_value));
				batValue.current = Get_ADC_Current(
						Get_ADC_MA_Vref(adc_current_value));
				sprintf((char*) usart.tx_data, "%d,%1.3f,%1.3f,%1.3f\n",
						relayStatus, batValue.voltage, batValue.current,
						batValue.temp);
			}
			if (falan) {
				Calibration_ADC();
				falan = false;
			}
			break;

		}
		// END CODE OF READING ADC USING MOVING AVERAGE METHOD (WITHOUT DMA) //

		// UART RECEIVE DATA WHILE CODE //
		if (usart.received) {
			UART_Cmd(usart.rx_data);
			Bat_Relay_Cmd(relayStatus);
			usart.received = false;
		}
		// END CODE UART RECEIVE DATA WHILE CODE //
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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


  /* USER CODE BEGIN ADC1_Init 1 */
  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */

  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */
  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */
  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 42000-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  /* USER CODE END TIM1_Init 2 */

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
  huart1.Init.BaudRate = 2000000;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RELAY_1_Pin|RELAY_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RELAY_1_Pin RELAY_2_Pin */
  GPIO_InitStruct.Pin = RELAY_1_Pin|RELAY_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
	while (1) {
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
