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
#include "stdlib.h"
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct __Sep_int_dec
{
	uint8_t integer;
	uint8_t decimal;
}Sep_int_dec;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NSS_HIGH() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)
#define NSS_LOW() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
CAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8] = {0};
uint32_t TxMailbox; //CAN controller to send data to the CAN bus -> transmits data according to the CAN protocol

CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8] = {0};

CAN_FilterTypeDef CAN_Filter;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */
uint8_t flag, flag_CAN, flag_cal, count;
uint8_t dt, i, delayCount = 20, E_ADC_buffer[3] = {0};
uint32_t time[2] = {0}, ADC18_Data, Sum, sample[250] = {0}, sample_data;
float SOC = 100, Rated_cap = 4300 * 6; //Battery rated capacity(mAh)
uint8_t len = sizeof(sample) / sizeof(uint32_t), Peak_Current;
double Current, Colomb_count, val[2] = {0}, integral_val; //Integral sum[0] : ADC1, Integral sum[1] : ADC2
uint8_t CAN_state;
uint32_t CAN_error;
Sep_int_dec SOC_CAN, Current_CAN;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM6_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
uint16_t Cal_dt(uint32_t t1, uint32_t t2);
uint32_t Moving_avg(uint32_t *ptr_arr_sample, uint32_t *ptr_sum, uint8_t i, uint8_t len, uint32_t n_data); //Moving average filter
void Seperation_int_and_dec(float num, Sep_int_dec *ptr_Data_CAN);
float Sel_ADC_Ch(uint16_t *adc_val);
double Round_dw(double data);
void E_ADC_SET(void);
void E_ADC_CNV(volatile uint8_t delayCount);
void E_ADC_S_D(void);
void E_ADC_W_U(void);
void CAN_TX_Config(void);
void CAN_Filter_Config(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin); //External interrupt
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim); //ADC1 - timer3(1ms) & timer4(10ms) - Moving average filter

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	CAN_Filter_Config();
	CAN_TX_Config();

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
  MX_CAN1_Init();
  MX_TIM6_Init();
  MX_SPI2_Init();
  MX_TIM7_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  //---CAN setting---//
  HAL_CAN_ConfigFilter(&hcan1, &CAN_Filter);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); //CAN Rx interrupt activation
  HAL_CAN_Start(&hcan1);

  //---DMA mode ADC---//
  HAL_Delay(10);
  E_ADC_SET();
  HAL_Delay(10);
  HAL_TIM_Base_Start_IT(&htim6); //Current
  HAL_TIM_Base_Start_IT(&htim7); //Colomb count
  HAL_TIM_Base_Start_IT(&htim3); //CAN
  HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  switch(flag)
	  {
	  case 1:
		  HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
		  HAL_NVIC_DisableIRQ(TIM3_IRQn);

		  E_ADC_CNV(delayCount);
		  sample_data = Moving_avg(sample, &Sum, i, len, ADC18_Data);
		  Current = Round_dw(((double)sample_data - 41735) / 845.8 + 0.01) - 0.03;
		  i++; //i : 0 ~ len-1 -> (odd - even)
		  if(i >= len) i = 0;

		  if(Peak_Current < Current_CAN.integer) Peak_Current = Current_CAN.integer;
		  //if((signed int)round(Current) >= Peak_Current) Peak_Current = (signed int)round(Current);

		  HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
		  HAL_NVIC_EnableIRQ(TIM3_IRQn);
		  flag = 0;
		  break;
	  }

	  //---Colomb_counting : ADC1(Timer3)---//
	  switch(RxData[0])
	  {
	  case 6:
		  if(flag_cal == 1)
		  {
			  if(count % 2 == 0) val[0] = Current, time[0] = HAL_GetTick();
			  else val[1] = Current, time[1] = HAL_GetTick();
			  dt = Cal_dt(time[0], time[1]);

			  integral_val = ((val[0] + val[1]) * dt) / 7200; //(mAh)
			  Colomb_count += integral_val;
			  SOC = (Rated_cap - Colomb_count) / Rated_cap * 100; //State of charge - SOC(%)

			  count++;
			  if(count >= len) count = 0;

			  flag_cal = 0;
		  }
		  break;
	  }

	  //---CAN---//
	  switch(flag_CAN)
	  {
	  case 1:
		  Seperation_int_and_dec(SOC, &SOC_CAN);
		  Seperation_int_and_dec(Current, &Current_CAN);

		  TxData[0] = SOC_CAN.integer;
		  TxData[1] = SOC_CAN.decimal;
		  TxData[2] = Current_CAN.integer;
		  TxData[3] = Current_CAN.decimal;
		  TxData[4] = Peak_Current;
		  for(int i = 5; i < 8; i++) TxData[i] = 0;

		  TxHeader.StdId = 0x09; //ID of message
		  TxMailbox = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
		  HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);

		  flag_CAN = 0;
		  break;
	  }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 5;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 500-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 100-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 180-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 10000-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 100-1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) //External interrupt
{
	flag_cal++;
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData);
}

uint16_t Cal_dt(uint32_t t1, uint32_t t2) //Time interval calculation
{
	uint16_t dt = 0;
	dt = abs(t2 - t1);
	if(dt > 100) dt = 0; //Initial value clearing(Initial condition setting & Overflow processing)
	return dt;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) //ADC1 - timer3(1ms) & timer4(10ms) - Moving average filter
{
	if(htim->Instance == TIM6) flag = 1;
	else if(htim->Instance == TIM7) flag_CAN = 1;
	else if(htim->Instance == TIM3) flag_cal = 1;
}

uint32_t Moving_avg(uint32_t *ptr_arr_sample, uint32_t *ptr_sum, uint8_t i, uint8_t len, uint32_t n_data) //Moving average filter
{
	*ptr_sum = *ptr_sum - ptr_arr_sample[i] + n_data; //sample[i] : old data, n_data : new data
	ptr_arr_sample[i] = n_data; //Data freshening
	return *ptr_sum / len; //##Filtered real-time data##
}

void Seperation_int_and_dec(float num, Sep_int_dec *ptr_Data_CAN)
{
	ptr_Data_CAN -> integer = (uint8_t)num;
	ptr_Data_CAN -> decimal = (uint8_t)((num - (uint8_t)num) * 100);
}

float Sel_ADC_Ch(uint16_t *adc_val)
{
	uint32_t result = 0;
	float Current = 0;

	for(int ch = 2; ch >= 0; ch--)
	{
		if(adc_val[ch] < 4080)
		{
			result = adc_val[ch];
			break;
		}
	}
	if(result == adc_val[0]) result = ADC18_Data;

	if(result == adc_val[2]) Current = (float)(result - 2140) / 100;
	else if(result == adc_val[1]) Current = (float)(result - 2070) / 20;
	else Current = (float)(result - 2050) / 4.1;

	return Current;
}

double Round_dw(double data)
{
	double result = 0;
	return result = round(data * 100) / 100;
}

void E_ADC_SET(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); //SDI -> pull-up
	HAL_Delay(1);
	NSS_HIGH();
}
void E_ADC_CNV(volatile uint8_t delayCount)
{
	NSS_LOW();
	HAL_SPI_Receive(&hspi2, (uint8_t *)E_ADC_buffer, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi2, (uint8_t *)&E_ADC_buffer[1], 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&hspi2, (uint8_t *)&E_ADC_buffer[2], 1, HAL_MAX_DELAY);
	NSS_HIGH();
	for(uint32_t i = 0; i < delayCount; i++)
	{
			 //tconv delay
	}

	E_ADC_buffer[2] = E_ADC_buffer[2] >> 6;
	ADC18_Data = E_ADC_buffer[0] << 10 | E_ADC_buffer[1] << 2 | E_ADC_buffer[2];
}
void E_ADC_S_D(void)
{
	HAL_SPI_DeInit(&hspi2);

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct); //SCK -> GPIO control

	NSS_HIGH();
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET); //SCK -> HIGH;
	HAL_Delay(1);
	NSS_LOW(); //shut-down
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); //SDI -> pull-down
}
void E_ADC_W_U(void)
{
	HAL_SPI_Init(&hspi2); //SCK -> LOW
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); //SDI -> pull-up
	NSS_HIGH(); //CS mode
	HAL_Delay(1);
	NSS_LOW(); //wake-up
	NSS_HIGH();
}

void CAN_TX_Config(void)
{
	TxHeader.DLC = 8;
	TxHeader.IDE = CAN_ID_STD; //2.0A
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.TransmitGlobalTime = DISABLE;
}

void CAN_Filter_Config(void)
{
	CAN_Filter.FilterActivation = ENABLE; //Filter : Enable -> message reception
	CAN_Filter.FilterIdHigh = 0x0AA << 5;
	CAN_Filter.FilterIdLow = 0x0AA << 5;
	CAN_Filter.FilterMaskIdHigh = 0x00 << 5;
	CAN_Filter.FilterMaskIdLow = 0x00 << 5;
	CAN_Filter.FilterScale = CAN_FILTERSCALE_16BIT; //2.0A -> 16Bit, 2.0B -> 32Bit
	CAN_Filter.FilterBank = 0;
	CAN_Filter.FilterFIFOAssignment = CAN_RX_FIFO0;
	CAN_Filter.FilterMode = CAN_FILTERMODE_IDLIST;
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
