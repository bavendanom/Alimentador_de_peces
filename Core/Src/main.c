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
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "ds1307.h"
#include "ds18b20.h"
#include "stdbool.h"
#include "stdio.h"
#include "stdarg.h"

//#include "command_handler.h"

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

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

#define BUFFER_SIZE 50

char buffer[16];
char transmit_text[ 64 ];
extern uint8_t read_flash_Byte[ 255 ];
volatile uint8_t address_to_write[3] ;
uint8_t things_to_write[255];
uint8_t size_to_send;

char uart_buffer[BUFFER_SIZE];  // Buffer para recepción UART
uint8_t uart_index = 0;         // Índice del buffer
uint8_t received_char;   // Variable temporal para recibir caracteres

char read_buffer[4];

int min_red = 0, max_red = 100;
int min_green = 0, max_green = 100;
int min_blue = 0, max_blue = 100;
int time_RTC = 0;
float temperature = 0;
float temperatura_promedio;
char value_to_write[4];


uint16_t counter_2_5min;
uint8_t counter_1min;
uint16_t counter10s;
uint16_t counter2_5s;
uint8_t counteraux;

int sec = 0, min = 0, hour = 0, dow = 0, dom = 0, month = 0, year = 0;


typedef struct {
    int min_red;
    int max_red;
    int min_green;
    int max_green;
    int min_blue;
    int max_blue;
} FlashParams;

FlashParams config; // Variable global para almacenar los valores

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void process_command(char *command);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    // Verifica qué Timer generó la interrupción
    if (htim->Instance == TIM14) {  // TIMx es el número del Timer que estás usando
    	//Temperature_To_Uart();
    	counter2_5s++;
    	counteraux++;

    if ( counter2_5s % 4 == 0 ){
    	counter10s++;

    	}
    if (counter2_5s % 60 == 0) {
		counter_2_5min++;
	}
    if (counter2_5s % 24 ==0){
    	counter_1min++;

    }
   }
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
    	if (received_char == '\n') {
			uart_buffer[uart_index] = '\0';
			process_command(uart_buffer);
			memset(uart_buffer, 0, BUFFER_SIZE); // Limpiar buffer
			uart_index = 0;
        } else if (received_char != '\r') { // Ignorar '\r'
            if (uart_index < BUFFER_SIZE - 1) {
                uart_buffer[uart_index++] = received_char;
            }
        }
        HAL_UART_Receive_IT(&huart1, &received_char, 1); // Reactivar recepción
    }
}


void process_command(char *command) {
    char param[15];
    int value;
    char debug_msg[50];
    char time_str[15];
    int sec, min, hour, dow, dom, month, year;

    // Limpiar el comando (eliminar \r, \n y espacios)
    char *clean_cmd = strtok(command, "\r\n ");
    if (clean_cmd == NULL) {
        HAL_UART_Transmit(&huart1, (uint8_t *)"ERROR: Comando vacio\n", strlen("ERROR: Comando vacio\r\n"), 100);
        return;
    }

    if (sscanf(clean_cmd, "%9[^$]$%d", param, &value) == 2) {
        // Confirmar recepción del comando
        snprintf(debug_msg, sizeof(debug_msg), "CMD: %s=%d\r\n", param, value);
        HAL_UART_Transmit(&huart1, (uint8_t *)debug_msg, strlen(debug_msg), 100);

        if (strcmp(param, "MIN_RED") == 0) {
            min_red = value;
            HAL_UART_Transmit(&huart1, (uint8_t *)"OK_MIN_RED\r\n", strlen("OK_MIN_RED\r\n"), 100);
            snprintf(debug_msg, sizeof(debug_msg), "SET MIN RED: %d\r\n", min_red);
            HAL_UART_Transmit(&huart1, (uint8_t *)debug_msg, strlen(debug_msg), 100); // Usar strlen

            config.min_red = value; // Actualizar estructura
            Flash_write_page(0x00, 0x00, 0x00, (uint8_t*)&config, sizeof(config)); // Guardar en Fla

        }else if (strcmp(param, "MAX_RED") == 0) {
            max_red = value;
            HAL_UART_Transmit(&huart1, (uint8_t *)"OK_MAX_RED\r\n", strlen("OK_MAX_RED\r\n"), 100);
            snprintf(debug_msg, sizeof(debug_msg), "SET MAX RED: %d\r\n", max_red);
            HAL_UART_Transmit(&huart1, (uint8_t *)debug_msg, strlen(debug_msg), 100); // Usar strlen

            config.max_red = value;
            Flash_write_page(0x00, 0x00, 0x00, (uint8_t*)&config, sizeof(config));

        }else if (strcmp(param, "MIN_GREEN") == 0) {
            min_green = value;
            HAL_UART_Transmit(&huart1, (uint8_t *)"OK_MAX_GREEN\r\n", strlen("OK_MAX_GREEN\r\n"), 100);
            snprintf(debug_msg, sizeof(debug_msg), "SET MIN GREEN: %d\r\n", min_green);
            HAL_UART_Transmit(&huart1, (uint8_t *)debug_msg, strlen(debug_msg), 100); // Usar strlen

            config.min_green = value;
            Flash_write_page(0x00, 0x00, 0x00, (uint8_t*)&config, sizeof(config));

        }else if (strcmp(param, "MAX_GREEN") == 0) {
            max_green = value;
            HAL_UART_Transmit(&huart1, (uint8_t *)"OK_MAX_GREEN\r\n", strlen("OK_MAX_GREEN\r\n"), 100);
            snprintf(debug_msg, sizeof(debug_msg), "SET MAX GREEN: %d\r\n", max_green);
            HAL_UART_Transmit(&huart1, (uint8_t *)debug_msg, strlen(debug_msg), 100); // Usar strlen

            config.max_green = value;
            Flash_write_page(0x00, 0x00, 0x00, (uint8_t*)&config, sizeof(config));

        }else if (strcmp(param, "MIN_BLUE") == 0) {
            min_blue = value;
            HAL_UART_Transmit(&huart1, (uint8_t *)"OK_MAX_BLUE\r\n", strlen("OK_MAX_BLUE\r\n"), 100);
            snprintf(debug_msg, sizeof(debug_msg), "SET MIN BLUE: %d\r\n", min_blue);
            HAL_UART_Transmit(&huart1, (uint8_t *)debug_msg, strlen(debug_msg), 100); // Usar strlen

            config.min_blue = value;
            Flash_write_page(0x00, 0x00, 0x00, (uint8_t*)&config, sizeof(config));

        }else if (strcmp(param, "MAX_BLUE") == 0) {
            max_blue = value;
            HAL_UART_Transmit(&huart1, (uint8_t *)"OK_MAX_BLUE\r\n", strlen("OK_MAX_BLUE\r\n"), 100);
            snprintf(debug_msg, sizeof(debug_msg), "SET MAX BLUE: %d\r\n", max_blue);
            HAL_UART_Transmit(&huart1, (uint8_t *)debug_msg, strlen(debug_msg), 100); // Usar strlen

            config.max_blue = value;
            Flash_write_page(0x00, 0x00, 0x00, (uint8_t*)&config, sizeof(config));

        }
    }if (sscanf(command, "%9[^$]$%14s", param, time_str) == 2) {
        if (strcmp(param, "SET_TIME") == 0) {
            // Extraer cada campo de 2 dígitos
            if (sscanf(time_str, "%2d%2d%2d%2d%2d%2d%2d",
                &sec, &min, &hour, &dow, &dom, &month, &year) == 7) {

                // Validar rangos (ejemplo básico)
                if (sec < 0 || sec > 59 || min < 0 || min > 59 ||
                    hour < 0 || hour > 23 || dow < 1 || dow > 7 ||
                    dom < 1 || dom > 31 || month < 1 || month > 12) {
                    HAL_UART_Transmit(&huart1, (uint8_t *)"ERROR: Valores fuera de rango\r\n", 30, 100);
                    return;
                }

                // Configurar el RTC
                Set_Time(sec, min, hour, dow, dom, month, year);
                HAL_UART_Transmit(&huart1, (uint8_t *)"Hora configurada\r\n", 18, 100);
            } else {
                HAL_UART_Transmit(&huart1, (uint8_t *)"ERROR: Formato inválido\r\n", 25, 100);
            }
        }
    }else {
        HAL_UART_Transmit(&huart1, (uint8_t *)"ERROR: Formato invalido\n", strlen("ERROR: Formato invalido\n"), 100);
    }
}


void check_temperature(float temperature) {
    // Apagar todos los LEDs primero
    HAL_GPIO_WritePin(RGB_R_GPIO_Port, RGB_R_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RGB_G_GPIO_Port, RGB_G_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RGB_B_GPIO_Port, RGB_B_Pin, GPIO_PIN_RESET);

    // Encender LEDs según los rangos configurados
    if (temperature >= min_red && temperature <= max_red) {
        HAL_GPIO_WritePin(RGB_R_GPIO_Port, RGB_R_Pin, GPIO_PIN_SET); // Encender rojo
    }
    if (temperature >= min_green && temperature <= max_green) {
        HAL_GPIO_WritePin(RGB_G_GPIO_Port, RGB_G_Pin, GPIO_PIN_SET); // Encender verde
    }
    if (temperature >= min_blue && temperature <= max_blue) {
        HAL_GPIO_WritePin(RGB_B_GPIO_Port, RGB_B_Pin, GPIO_PIN_SET); // Encender azul
    }
}

void Print_Temperature_To_Uart(void){
	temperature = DS18B20_ReadTemperature();
	check_temperature(temperature);
	size_to_send = sprintf( transmit_text, "result temperature %.2f\r\n", temperature);
	transmit_text[size_to_send] = '\0';
	HAL_UART_Transmit(&huart1, (uint8_t *)transmit_text, size_to_send, 1000);
}

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
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start_IT(&htim14);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  Flash_read_identification_id();

  /*Esta funcion configura el reloj, solo es necesario correrla una sola vez
  (sec, min, hour, dow, dom, month, year)*/
  //Set_Time(00, 00, 01, 4, 6, 3, 25);



   HAL_UART_Receive_IT(&huart1, &received_char, 1); // Iniciar recepción UART

   Flash_page_erase(0x00, 0x00, 0x00);
   // Leer configuración desde Flash
	 Flash_read_page(0x00, 0x00, 0x00, sizeof(config)); // Dirección 0x000000
	 memcpy(&config, read_flash_Byte, sizeof(config));


	 if(config.min_red == 0xFFFFFFFF){ // Asume que 0xFFFFFFFF es el valor por defecto
	     config.min_red = 30; // Valor por defecto
	     config.max_red = 40;
	     config.min_green = 25;
	     config.max_green = 30;
	     config.min_blue = 20;
	     config.max_blue = 25;
	     Flash_write_page(0x00, 0x00, 0x00, (uint8_t*)&config, sizeof(config));
	 }

	 // Asignar valores a las variables globales
	 min_red = config.min_red;
	 max_red = config.max_red;
	 min_green = config.min_green;
	 max_green = config.max_green;
	 min_blue = config.min_blue;
	 max_blue = config.max_blue;

	 temperatura_promedio =0;
	 Print_Temperature_To_Uart();


  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	Get_Time();
	sprintf(buffer, "%02d:%02d:%02d\r\n", time.hour, time.minutes, time.seconds);
	HAL_UART_Transmit(&huart1,(uint8_t *)buffer, strlen(buffer), 100);
	HAL_Delay(1000);



	if (counter10s >=1) {
		Print_Temperature_To_Uart();
		counter10s = 0;
		temperatura_promedio = temperatura_promedio + temperature;

	 }
	if (counter_1min >= 1) {

		  temperatura_promedio = temperatura_promedio/6.0;
		  size_to_send = sprintf( transmit_text, "temperatura promedio guardada en flash: %.1f\r\n", temperatura_promedio);
		  HAL_UART_Transmit(&huart1, (uint8_t *)transmit_text, size_to_send, 1000);
		  sprintf(value_to_write, "%.1f", temperatura_promedio);
		  Flash_page_erase(0x00, 0x03, 0x00);
		  Flash_write_page(0x00,0x03,0x00,(uint8_t *)value_to_write,strlen(value_to_write));

		  counter_1min = 0;
		  temperatura_promedio = 0;
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
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 47999;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 2499;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

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
// Verificar si la Flash está vacía (valores iniciales)

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
