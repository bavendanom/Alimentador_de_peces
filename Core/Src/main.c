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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Flash.h"
#include <string.h>
#include <stdio.h>
#include "ds1307.h"
#include "ds18b20.h"

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
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;

//TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
char transmit_text[ 64 ];
extern uint8_t read_flash_Byte[ 255 ];
volatile uint8_t address_to_write[3] ;
uint8_t things_to_write[255];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char buffer[16];

///* Temperature sensor ---------------------------------------------------------*/
//void Set_Pin_Input (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin){
//	//HAL_GPIO_DeInit(GPIOx, GPIO_Pin);
//	GPIO_InitTypeDef GPIO_InitStruct = {0};
//	/*Configure GPIO pin : PA0 */
//	GPIO_InitStruct.Pin = GPIO_Pin;
//	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//	GPIO_InitStruct.Pull = GPIO_PULLUP;
//	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
//}
//
//void Set_Pin_Output (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin){
//	//HAL_GPIO_DeInit(GPIOx, GPIO_Pin);
//	GPIO_InitTypeDef GPIO_InitStruct = {0};
//	/*Configure GPIO pin : PA0 */
//	GPIO_InitStruct.Pin = GPIO_Pin;
//	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
//}
//
//
//void delay (uint16_t us)
//{
//	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
//	while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
//}
//
//uint8_t DS18B20_Start (void)
//{
//	uint8_t Response = 0;
//	Set_Pin_Output(Temperature_sensor_GPIO_Port, Temperature_sensor_Pin);   // set the pin as output
//	HAL_GPIO_WritePin (Temperature_sensor_GPIO_Port, Temperature_sensor_Pin, 0);  // pull the pin low
//	delay (480);   // delay according to datasheet
//
//	Set_Pin_Input(Temperature_sensor_GPIO_Port, Temperature_sensor_Pin);    // set the pin as input
//	delay (80);    // delay according to datasheet
//
//	if (!(HAL_GPIO_ReadPin (Temperature_sensor_GPIO_Port, Temperature_sensor_Pin))) Response = 1;    // if the pin is low i.e the presence pulse is detected
//	else Response = 0;
//
//	delay (400); // 480 us delay totally.
//
//	return Response;
//}
//void DS18B20_Write (uint8_t data)
//{
//	Set_Pin_Output(Temperature_sensor_GPIO_Port, Temperature_sensor_Pin);  // set as output
//
//	for (int i=0; i<8; i++)
//	{
//
//		if ((data & (1<<i))!=0)  // if the bit is high
//		{
//			// write 1
//
//			Set_Pin_Output(Temperature_sensor_GPIO_Port, Temperature_sensor_Pin);  // set as output
//			HAL_GPIO_WritePin (Temperature_sensor_GPIO_Port, Temperature_sensor_Pin, 0);  // pull the pin LOW
//			delay (1);  // wait for 1 us
//
//			Set_Pin_Input(Temperature_sensor_GPIO_Port, Temperature_sensor_Pin);  // set as input
//			delay (60);  // wait for 60 us
//		}
//
//		else  // if the bit is low
//		{
//			// write 0
//
//			Set_Pin_Output(Temperature_sensor_GPIO_Port, Temperature_sensor_Pin);
//			HAL_GPIO_WritePin (Temperature_sensor_GPIO_Port, Temperature_sensor_Pin, 0);  // pull the pin LOW
//			delay (60);  // wait for 60 us
//
//			Set_Pin_Input(Temperature_sensor_GPIO_Port, Temperature_sensor_Pin);
//		}
//	}
//}
//float convert_temperature(uint8_t byte_1, uint8_t byte_2 ){
//	uint16_t tempval = byte_2 << 8 | byte_1;
//	float result_temp = (128.0 / 2048)*tempval;
//
//	return  result_temp;
//
//}
//uint8_t DS18B20_Read (void)
//{
//	uint8_t value=0;
//	Set_Pin_Input(Temperature_sensor_GPIO_Port, Temperature_sensor_Pin);
//
//	for (int i=0;i<8;i++)
//	{
//		Set_Pin_Output(Temperature_sensor_GPIO_Port, Temperature_sensor_Pin);;   // set as output
//
//		HAL_GPIO_WritePin (GPIOA, GPIO_PIN_3, 0);  // pull the data pin LOW
//		delay (2);  // wait for 2 us
//
//		Set_Pin_Input(Temperature_sensor_GPIO_Port, Temperature_sensor_Pin);  // set as input
//		delay (5);  // wait for 2 us
//		if (HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_3))  // if the pin is HIGH
//		{
//			value |= 1<<i;  // read = 1
//		}
//		delay (60);  // wait for 60 us
//	}
//	return value;
//}
//
///* Temperature sensor ---------------------------------------------------------*/

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
  MX_ADC_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  Flash_read_identification_id();
//  Flash_activate_deactivate_block_protect();
//  uint8_t status_reg1;
//  uint8_t status_reg2;
//  uint8_t status_reg3;
//
//  Flash_read_status1_register( &status_reg1 );
//  Flash_read_status2_register( &status_reg2 );
//  Flash_read_status3_register( &status_reg3 );
//
//  Flash_read_page(0x00, 0x01, 0x00, 255);
//  Flash_sector_erase(0x00, 0x01, 0x02);
//  Flash_read_page(0x00, 0x01, 0x00, 255);
//  //uint8_t size_to_send;
//  //size_to_send = sprintf( (char*)things_to_write, "the temperature is: %d \r\n", 30 );
//  sprintf( (char*)things_to_write, "the temperature is: %d \r\n", 30 );
//  Flash_write_page( 0x00, 0x01, 0x02 , &things_to_write[0] , 255 );
//  Flash_read_page(0x00, 0x01, 0x02, 255);
//  Flash_read_page(0x00, 0x01, 0x02, 255);


  // Esta funcion configura el reloj, solo es necesario correrla una sola vez
  //(sec, min, hour, dow, dom, month, year)
  //Set_Time(00, 00, 01, 4, 6, 3, 25);

  	//HAL_UART_Transmit(&huart2, "hi", 2, 1000);
//    uint8_t Presence;
//    uint8_t Temp_byte1;
//    uint8_t Temp_byte2;
    uint8_t size_to_send;
//    float  temp_dec;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	Get_Time();
	sprintf(buffer, "%02d:%02d:%02d\r\n", time.hour, time.minutes, time.seconds);
	HAL_UART_Transmit(&huart1,(uint8_t *)buffer, strlen(buffer), 100);
	HAL_Delay(1000);

//	Presence = DS18B20_Start();
//	HAL_Delay (1);
//	DS18B20_Write (0xCC);  // skip ROM
//	DS18B20_Write (0x44);  // convert t
//	HAL_Delay (800);
//
//	Presence = DS18B20_Start ();
//	HAL_Delay(1);
//	DS18B20_Write (0xCC);  // skip ROM
//	DS18B20_Write (0xBE);  // Read Scratch-pad
//
//	Temp_byte1 = DS18B20_Read();
//	Temp_byte2 = DS18B20_Read();
//
//	//HAL_UART_Transmit(&huart1,(uint8_t *) "read \r\n", 9, 1000);
//	size_to_send = sprintf( transmit_text, "presence %d el primer %d y el segundo %d \r\n", Presence,Temp_byte1,Temp_byte2);
//	transmit_text[size_to_send] = '\0';
//	HAL_UART_Transmit(&huart1, (uint8_t *)transmit_text, size_to_send, 1000);
//	temp_dec = convert_temperature(Temp_byte1 , Temp_byte2);

	float temperature = DS18B20_ReadTemperature();
	size_to_send = sprintf( transmit_text, "result temperature %.2f\r\n", temperature);
	transmit_text[size_to_send] = '\0';
	HAL_UART_Transmit(&huart1, (uint8_t *)transmit_text, size_to_send, 1000);

	HAL_Delay(1000);


	HAL_GPIO_TogglePin(Alerta_voltaje_min_GPIO_Port, Alerta_voltaje_min_Pin);
	HAL_Delay(1000);
	HAL_GPIO_TogglePin(RGB_R_GPIO_Port, RGB_R_Pin);
	HAL_Delay(1000);
	HAL_GPIO_TogglePin(RGB_G_GPIO_Port, RGB_G_Pin);
	HAL_Delay(1000);
	HAL_GPIO_TogglePin(RGB_B_GPIO_Port, RGB_B_Pin);
	HAL_Delay(1000);


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
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
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10805D88;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  htim1.Init.Prescaler = 47;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RGB_R_Pin|RGB_G_Pin|RGB_B_Pin|Temperature_sensor_Pin
                          |Alerta_voltaje_min_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RGB_R_Pin RGB_G_Pin RGB_B_Pin Temperature_sensor_Pin
                           Alerta_voltaje_min_Pin */
  GPIO_InitStruct.Pin = RGB_R_Pin|RGB_G_Pin|RGB_B_Pin|Temperature_sensor_Pin
                          |Alerta_voltaje_min_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_CS_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI2_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Motor_control_Pin */
  GPIO_InitStruct.Pin = Motor_control_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM3;
  HAL_GPIO_Init(Motor_control_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Boton_1_Pin Boton_2_Pin */
  GPIO_InitStruct.Pin = Boton_1_Pin|Boton_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
