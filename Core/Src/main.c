/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
uint8_t segmentPatterns[16] = {
		0x3F, // 0: 0111111
		0x06, // 1: 0000110
		0x5B, // 2: 1011011
		0x4F, // 3: 1001111
		0x66, // 4: 1100110
		0x6D, // 5: 1101101
		0x7D, // 6: 1111101
		0x07, // 7: 0000111
		0x7F, // 8: 1111111
		0x6F, // 9: 1101111
		0x77, // A: 1110111
		0x7C, // B: 1111100
		0x39, // C: 0111001
		0x5E, // D: 1011110
		0x79, // E: 1111001
		0x71  // F: 1110001
};
void litBit(uint8_t data){
	HAL_GPIO_WritePin(SEG_A_GPIO_Port, SEG_A_Pin, (data & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SEG_B_GPIO_Port, SEG_B_Pin, (data & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SEG_C_GPIO_Port, SEG_C_Pin, (data & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SEG_D_GPIO_Port, SEG_D_Pin, (data & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SEG_E_GPIO_Port, SEG_E_Pin, (data & 0x10) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SEG_F_GPIO_Port, SEG_F_Pin, (data & 0x20) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SEG_G_GPIO_Port, SEG_G_Pin, (data & 0x40) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SEG_DP_GPIO_Port, SEG_DP_Pin, (data & 0x80) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void displayNumber(uint8_t number){
	if (0 <= number && number <= 0x0F){
		litBit(segmentPatterns[number]);
	}
}

void enableDigit(uint8_t digit){
	switch(digit){
	case 0:
		HAL_GPIO_WritePin(G1_GPIO_Port, G1_Pin, GPIO_PIN_SET);
		break;
	case 1:
		HAL_GPIO_WritePin(G2_GPIO_Port, G2_Pin, GPIO_PIN_SET);
		break;
	case 2:
		HAL_GPIO_WritePin(G3_GPIO_Port, G3_Pin, GPIO_PIN_SET);
		break;
	case 3:
		HAL_GPIO_WritePin(G4_GPIO_Port, G4_Pin, GPIO_PIN_SET);
		break;
	case 4:
		HAL_GPIO_WritePin(G5_GPIO_Port, G5_Pin, GPIO_PIN_SET);
		break;
	case 5:
		HAL_GPIO_WritePin(G6_GPIO_Port, G6_Pin, GPIO_PIN_SET);
		break;
	}
}

void disableDigit(uint8_t digit){
	switch(digit){
	case 0:
		HAL_GPIO_WritePin(G1_GPIO_Port, G1_Pin, GPIO_PIN_RESET);
		break;
	case 1:
		HAL_GPIO_WritePin(G2_GPIO_Port, G2_Pin, GPIO_PIN_RESET);
		break;
	case 2:
		HAL_GPIO_WritePin(G3_GPIO_Port, G3_Pin, GPIO_PIN_RESET);
		break;
	case 3:
		HAL_GPIO_WritePin(G4_GPIO_Port, G4_Pin, GPIO_PIN_RESET);
		break;
	case 4:
		HAL_GPIO_WritePin(G5_GPIO_Port, G5_Pin, GPIO_PIN_RESET);
		break;
	case 5:
		HAL_GPIO_WritePin(G6_GPIO_Port, G6_Pin, GPIO_PIN_RESET);
		break;
	}
}

void disableAllDigit(){
	HAL_GPIO_WritePin(G1_GPIO_Port, G1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(G2_GPIO_Port, G2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(G3_GPIO_Port, G3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(G4_GPIO_Port, G4_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(G5_GPIO_Port, G5_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(G6_GPIO_Port, G6_Pin, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(SEG_A_GPIO_Port, SEG_A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SEG_B_GPIO_Port, SEG_B_Pin,  GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SEG_C_GPIO_Port, SEG_C_Pin,  GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SEG_D_GPIO_Port, SEG_D_Pin,  GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SEG_E_GPIO_Port, SEG_E_Pin,  GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SEG_F_GPIO_Port, SEG_F_Pin,  GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SEG_G_GPIO_Port, SEG_G_Pin,  GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SEG_DP_GPIO_Port, SEG_DP_Pin,  GPIO_PIN_RESET);
}

void Delay_us(uint16_t us)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0);  // Set the counter value to 0
    HAL_TIM_Base_Start(&htim2);  // Start the timer
    while (__HAL_TIM_GET_COUNTER(&htim2) < us);  // Wait until the counter reaches the us input value
    HAL_TIM_Base_Stop(&htim2);  // Stop the timer
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);
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
  MX_RTC_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uint8_t ct = 0;
	RTC_DateTypeDef gDate;
	RTC_TimeTypeDef gTime;
	disableAllDigit();
#define DIGIT_DELAY 1
#define DIGIT_DELAY_US 200
#define DISABLE_DELAY_US 10
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BCD);


		disableDigit(5); enableDigit(0);
		displayNumber(gTime.Hours / 10);
		Delay_us(DIGIT_DELAY_US);
		disableAllDigit();
		Delay_us(DISABLE_DELAY_US);


		disableDigit(0); enableDigit(1);
		displayNumber(gTime.Hours % 10);
		Delay_us(DIGIT_DELAY_US);
		disableAllDigit();
		Delay_us(DISABLE_DELAY_US);


		disableDigit(1); enableDigit(2);
		displayNumber(gTime.Minutes / 10);
		Delay_us(DIGIT_DELAY_US);
		disableAllDigit();
		Delay_us(DISABLE_DELAY_US);


		disableDigit(2); enableDigit(3);
		displayNumber(gTime.Minutes % 10);
		Delay_us(DIGIT_DELAY_US);
		disableAllDigit();
		Delay_us(DISABLE_DELAY_US);


		disableDigit(3); enableDigit(4);
		displayNumber(gTime.Seconds / 10);
		Delay_us(DIGIT_DELAY_US);
		disableAllDigit();
		Delay_us(DISABLE_DELAY_US);


		disableDigit(4); enableDigit(5);
		displayNumber(gTime.Seconds % 10);
		Delay_us(DIGIT_DELAY_US);
		disableAllDigit();
		Delay_us(DISABLE_DELAY_US);


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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
#define MAGIC_NO 0x1208

	if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) != MAGIC_NO)
	{
		RTC_TimeTypeDef sTime = {0};
		RTC_DateTypeDef sDate = {0};

		sTime.Hours = 15;
		sTime.Minutes = 17;
		sTime.Seconds = 50;
		if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
		{
			Error_Handler();
		}
		sDate.WeekDay = RTC_WEEKDAY_SATURDAY;
		sDate.Month = RTC_MONTH_JUNE;
		sDate.Date = 06;
		sDate.Year = 24;
		if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
		{
			Error_Handler();

		}
		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, MAGIC_NO);
	}
  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 79;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, G6_Pin|SEG_E_Pin|G2_Pin|SEG_B_Pin
                          |SEG_A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, G5_Pin|SEG_DP_Pin|G4_Pin|SEG_MINUS_Pin
                          |SEG_G_Pin|SEG_F_Pin|G3_Pin|SEG_C_Pin
                          |SEG_D_Pin|LED1_Pin|LED2_Pin|LED3_Pin
                          |LED4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(G1_GPIO_Port, G1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : G6_Pin SEG_E_Pin G2_Pin SEG_B_Pin
                           SEG_A_Pin */
  GPIO_InitStruct.Pin = G6_Pin|SEG_E_Pin|G2_Pin|SEG_B_Pin
                          |SEG_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : G5_Pin SEG_DP_Pin G4_Pin SEG_MINUS_Pin
                           SEG_G_Pin SEG_F_Pin G3_Pin SEG_C_Pin
                           SEG_D_Pin LED1_Pin LED2_Pin LED3_Pin
                           LED4_Pin */
  GPIO_InitStruct.Pin = G5_Pin|SEG_DP_Pin|G4_Pin|SEG_MINUS_Pin
                          |SEG_G_Pin|SEG_F_Pin|G3_Pin|SEG_C_Pin
                          |SEG_D_Pin|LED1_Pin|LED2_Pin|LED3_Pin
                          |LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : G1_Pin */
  GPIO_InitStruct.Pin = G1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(G1_GPIO_Port, &GPIO_InitStruct);

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
