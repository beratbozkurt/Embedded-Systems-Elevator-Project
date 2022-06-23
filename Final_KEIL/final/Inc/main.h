/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void update_floor(void);
void open_door(void);
void close_door(void);
void update_up_down(void);
void run_buzzer(void);
void run_floor_logic(void);
void run_door_logic(void);
uint8_t find_arrival_target(void);
void run_smart_movement_algorithm(uint16_t floor);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define elevator_speed_Pin GPIO_PIN_0
#define elevator_speed_GPIO_Port GPIOA
#define LCD_EN_Pin GPIO_PIN_1
#define LCD_EN_GPIO_Port GPIOA
#define LCD_RS_Pin GPIO_PIN_2
#define LCD_RS_GPIO_Port GPIOA
#define LCD_D4_Pin GPIO_PIN_4
#define LCD_D4_GPIO_Port GPIOA
#define LCD_D5_Pin GPIO_PIN_5
#define LCD_D5_GPIO_Port GPIOA
#define LCD_D6_Pin GPIO_PIN_6
#define LCD_D6_GPIO_Port GPIOA
#define LCD_D7_Pin GPIO_PIN_7
#define LCD_D7_GPIO_Port GPIOA
#define elevator_down_Pin GPIO_PIN_2
#define elevator_down_GPIO_Port GPIOB
#define flame_sensor_Pin GPIO_PIN_10
#define flame_sensor_GPIO_Port GPIOB
#define flame_sensor_EXTI_IRQn EXTI15_10_IRQn
#define gas_sensor_Pin GPIO_PIN_11
#define gas_sensor_GPIO_Port GPIOB
#define gas_sensor_EXTI_IRQn EXTI15_10_IRQn
#define elevator_up_Pin GPIO_PIN_12
#define elevator_up_GPIO_Port GPIOB
#define floor_zero_button_Pin GPIO_PIN_13
#define floor_zero_button_GPIO_Port GPIOB
#define floor_zero_button_EXTI_IRQn EXTI15_10_IRQn
#define floor_one_button_Pin GPIO_PIN_14
#define floor_one_button_GPIO_Port GPIOB
#define floor_one_button_EXTI_IRQn EXTI15_10_IRQn
#define floor_two_button_Pin GPIO_PIN_15
#define floor_two_button_GPIO_Port GPIOB
#define floor_two_button_EXTI_IRQn EXTI15_10_IRQn
#define buzzer_Pin GPIO_PIN_8
#define buzzer_GPIO_Port GPIOA
#define door_open_Pin GPIO_PIN_11
#define door_open_GPIO_Port GPIOA
#define door_close_Pin GPIO_PIN_12
#define door_close_GPIO_Port GPIOA
#define echo_Pin GPIO_PIN_3
#define echo_GPIO_Port GPIOB
#define trigger_Pin GPIO_PIN_4
#define trigger_GPIO_Port GPIOB
#define led_floor_zero_Pin GPIO_PIN_7
#define led_floor_zero_GPIO_Port GPIOB
#define led_floor_one_Pin GPIO_PIN_8
#define led_floor_one_GPIO_Port GPIOB
#define led_floor_two_Pin GPIO_PIN_9
#define led_floor_two_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
extern char data_transmit[256];
extern uint8_t data_receive[1];
extern int len;
extern uint16_t current_floor;
extern uint16_t target_floor_array[3];
extern uint8_t fan_control;
extern uint8_t is_flame_sensor;
extern uint8_t is_gas_sensor;
extern uint32_t adc_read;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
