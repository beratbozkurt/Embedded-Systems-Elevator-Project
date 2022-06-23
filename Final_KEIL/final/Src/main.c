/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stdio.h"
#include "LCD.h"
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
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE BEGIN PV */
uint16_t current_floor = 0;
uint16_t target_floor_array[3] = {0};
uint8_t up = 0;
uint8_t down = 0;
uint8_t door_request_array[3] = {0};
uint32_t adc_read = 2048;
uint8_t transmit_flag = 0;
uint16_t delay_percent;
uint16_t delay;
float distance = 0;

uint8_t last_ten_arrivals[10] = {NULL};
uint8_t ten_arrival_counters[3] = {0};
uint8_t arrival_counter = 0;
uint8_t arrival_target = 0;
uint8_t is_smart_move = 0;
uint8_t flag = 0;

uint8_t is_flame_sensor = 1;
uint8_t is_gas_sensor = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char data_transmit[256];
char data_floor[256];
uint8_t data_receive[1];
int len = 0;

void delay_us(uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim2,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim2) < us);  // wait for the counter to reach the us input in the parameter
}

float sensor_read()
{
	uint32_t time = 0;
	float distance_temp = 0;
	HAL_GPIO_WritePin(GPIOB, trigger_Pin, GPIO_PIN_RESET);
	delay_us(10);
	HAL_GPIO_WritePin(GPIOB, trigger_Pin, GPIO_PIN_SET);
	delay_us(10);
	HAL_GPIO_WritePin(GPIOB, trigger_Pin, GPIO_PIN_RESET);
	delay_us(10);
	while (!HAL_GPIO_ReadPin(GPIOB, echo_Pin));
	while (HAL_GPIO_ReadPin(GPIOB, echo_Pin))
	{
		time++;
		delay_us(1);
	}
	distance_temp = (float) time*0.87;
	delay_us(50);
	return distance_temp;
}

void init_system(void)
{
	distance = sensor_read();
	update_floor();
	close_door();
	data_receive[0] = 48;
	len = sprintf(data_transmit, "Floor: %d Distance: %f\r\n", data_receive[0] - 48, distance);
	delay_percent = 50;
	delay = (delay_percent * htim2.Init.Period) / 100;
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
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim2);
	lcd_init(_LCD_4BIT, _LCD_FONT_5x8, _LCD_2LINE);
	lcd_print(1,1,"0");
	lcd_print(2,1,"-");
	HAL_ADC_Start_IT(&hadc1);
	HAL_ADCEx_Calibration_Start(&hadc1);
	
	HAL_UART_Receive_IT(&huart1, data_receive, sizeof(data_receive));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	init_system();
  while (1)
  {
		if (is_flame_sensor || is_gas_sensor) {
			
			if (is_flame_sensor && !is_gas_sensor) {
				lcd_print(1,1,"FLAME    ");
				lcd_print(2,1,"DETECTED");
			} else if (!is_flame_sensor && is_gas_sensor) {
				lcd_print(1,1,"GAS      ");
				lcd_print(2,1,"DETECTED");
			} else {
				lcd_print(1,1,"FLAME-GAS");
				lcd_print(2,1,"DETECTED");
			}
			
			delay_us(htim2.Init.Period / 3);
			continue;
		}
		
		delay_percent = (adc_read * 100)/4096;
		delay = (delay_percent * htim2.Init.Period) / 100;
		
		distance = sensor_read();
		if (distance == NULL || distance > 45 || distance < 0)
		{
			distance = 45.1;	 
			len = sprintf(data_transmit, "Distance: %f\r\nDistance values must be between 0 and 45 cm\r\nControl the distance sensor, Door will be open till sensor is corrected!\r\n", distance);
		} 
		

		run_floor_logic();		
		
		update_up_down();
		delay_us(delay);
		update_floor();
		
		run_door_logic();
		
		is_smart_move =  target_floor_array[0] == 0 &&  target_floor_array[1] == 0 &&  target_floor_array[2] == 0;
		if (is_smart_move) { 
			arrival_target = find_arrival_target();
			is_smart_move = is_smart_move && (current_floor != arrival_target);
			if (is_smart_move) {
				target_floor_array[arrival_target] = 2;
				delay_us(htim2.Init.Period / 3);
			}
		}
		
		
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
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
  htim1.Init.Prescaler = 124;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 63999;
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
  htim2.Init.Prescaler = 374;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 63999;
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
  huart1.Init.BaudRate = 9600;
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
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_EN_Pin|LCD_RS_Pin|LCD_D4_Pin|LCD_D5_Pin
                          |LCD_D6_Pin|LCD_D7_Pin|buzzer_Pin|door_open_Pin
                          |door_close_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, elevator_down_Pin|elevator_up_Pin|trigger_Pin|led_floor_zero_Pin
                          |led_floor_one_Pin|led_floor_two_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LCD_EN_Pin LCD_RS_Pin LCD_D4_Pin LCD_D5_Pin
                           LCD_D6_Pin LCD_D7_Pin buzzer_Pin door_open_Pin
                           door_close_Pin */
  GPIO_InitStruct.Pin = LCD_EN_Pin|LCD_RS_Pin|LCD_D4_Pin|LCD_D5_Pin
                          |LCD_D6_Pin|LCD_D7_Pin|buzzer_Pin|door_open_Pin
                          |door_close_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : elevator_down_Pin elevator_up_Pin trigger_Pin led_floor_zero_Pin
                           led_floor_one_Pin led_floor_two_Pin */
  GPIO_InitStruct.Pin = elevator_down_Pin|elevator_up_Pin|trigger_Pin|led_floor_zero_Pin
                          |led_floor_one_Pin|led_floor_two_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : flame_sensor_Pin gas_sensor_Pin */
  GPIO_InitStruct.Pin = flame_sensor_Pin|gas_sensor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : floor_zero_button_Pin floor_one_button_Pin floor_two_button_Pin */
  GPIO_InitStruct.Pin = floor_zero_button_Pin|floor_one_button_Pin|floor_two_button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : echo_Pin */
  GPIO_InitStruct.Pin = echo_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(echo_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	
	HAL_UART_Receive_IT(&huart1, (uint8_t*) data_receive, sizeof(data_receive));
		
		if (data_receive[0] == 48) {
			target_floor_array[0] = 1;
		} else if (data_receive[0] == 49) {
			target_floor_array[1] = 1;
		} else if (data_receive[0] == 50) {
			target_floor_array[2] = 1;
		}
}

void run_floor_logic(void) {
	if(target_floor_array[0] || target_floor_array[0]==2){
			if (current_floor > 0) {
				current_floor--;
				down = 1;
			} else {
				if (target_floor_array[0] == 1 && target_floor_array[0] !=2 ) run_smart_movement_algorithm(0);
				target_floor_array[0] = 0;
				door_request_array[0] = 1;
			}
		} else if(target_floor_array[1] || target_floor_array[1]==2){
			if (current_floor == 0) {
				current_floor++;
				up = 1;
			} else if(current_floor==2){
				current_floor--;
				down = 1;
			} else {
				if (target_floor_array[1] == 1 && target_floor_array[1] !=2) run_smart_movement_algorithm(1);
				target_floor_array[1] = 0;
				door_request_array[1] = 1;
			}
		} else if (target_floor_array[2] || target_floor_array[2]==2) {
			if (current_floor < 2) {
				current_floor++;
				up = 1;
			} else {
				if (target_floor_array[2] == 1 && target_floor_array[2] != 2) run_smart_movement_algorithm(2);
				target_floor_array[2] = 0;
				door_request_array[2] = 1;
			}
		}
}

void run_door_logic(void) {
	if (door_request_array[current_floor]) {
			open_door();
			run_buzzer();
			
			while (distance < 40 || distance > 45) {
				distance = sensor_read();
				run_buzzer();
			}			
			
			close_door();
			door_request_array[current_floor] = 0;
		}
}

void run_smart_movement_algorithm(uint16_t floor) {
		if (flag) {
			int second_time = last_ten_arrivals[arrival_counter];
			ten_arrival_counters[second_time] = ten_arrival_counters[second_time] - 1;
		}
		last_ten_arrivals[arrival_counter++] = floor;
		ten_arrival_counters[floor]++;
		if (arrival_counter == 10) {
			arrival_counter = 0;
			flag = 1;
		}
}

uint8_t find_arrival_target(void) {
	uint8_t target = 2;
	for (int i = 2; i >= 0; i--) {
		if (ten_arrival_counters[i] >= ten_arrival_counters[target]) 
			target = (uint8_t) i;
	}
	
	return target;
}


void run_buzzer(void) {
	delay_us(htim2.Init.Period / 3);
	HAL_GPIO_WritePin(GPIOA, buzzer_Pin, GPIO_PIN_SET);
	delay_us(htim2.Init.Period / 3);
	HAL_GPIO_WritePin(GPIOA, buzzer_Pin, GPIO_PIN_RESET);
}

void update_floor(void) {	
	GPIO_PinState zero = current_floor == 0 ? GPIO_PIN_SET : GPIO_PIN_RESET;
	GPIO_PinState one = current_floor == 1 ? GPIO_PIN_SET : GPIO_PIN_RESET;
	GPIO_PinState two = current_floor == 2 ? GPIO_PIN_SET : GPIO_PIN_RESET;
	
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, zero);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, one);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, two);
	
	sprintf(data_floor, "%d        ", current_floor);
	lcd_print(1,1, data_floor);
	
	if (data_receive[0] < 48 || data_receive[0] > 50) {
		len = sprintf(data_transmit, "Avaliable floors are: 0 - 1 - 2\r\n");
	} else {
		len = sprintf(data_transmit, "\r\nFloor: %d Distance: %f\r\nArrival Target: %d\r\nLast ten arrivals: [%d, %d, %d]\r\nADC: %d\r\n", current_floor, distance, arrival_target, ten_arrival_counters[0], ten_arrival_counters[1], ten_arrival_counters[2], adc_read );
	}
	
	up = 0;
	down = 0;
}

void open_door(void) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
}

void close_door(void) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
}

void update_up_down(void) {
	GPIO_PinState up_state = up == 1 ? GPIO_PIN_SET : GPIO_PIN_RESET;
	GPIO_PinState down_state = down == 1 ? GPIO_PIN_SET : GPIO_PIN_RESET;
	
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, up_state);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, down_state);
	
	if (up) {
		lcd_print(2,1,"UP       ");
	} else if (down) {
		lcd_print(2,1,"DOWN     ");
	} else {
		lcd_print(2,1,"STAY     ");
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

