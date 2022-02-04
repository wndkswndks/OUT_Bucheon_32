/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum
{
	OFF = 0,
	ON = 1,
} ON_OFF_E;

typedef enum
{
	STEP1,
	STEP2,
	STEP3,
	STEP4,
} STEP_E;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

typedef struct
{
	uint8_t hour;
	uint8_t min;
	uint8_t sec;
} TIME_T;

typedef struct
{
	uint32_t cnt;
	uint32_t past_time;
	uint32_t past_time_small;
	uint8_t on_flag;
	STEP_E step;
	TIME_T dest_time;
} LED_T;



//#define RED_ON_TIME     46800
//#define RED_OFF_TIME    39600
//#define GREEN_ON_TIME   600
//#define GREEN_OFF_TIME  3000
//#define BLUE_ON_TIME    60
//#define BLUE_OFF_TIME   7140
//#define WHITE_ON_TIME   600
//#define WHITE_OFF_TIME  3000

//#define RED_ON_TIME     100
//#define RED_OFF_TIME    100
//#define GREEN_ON_TIME   30
//#define GREEN_OFF_TIME  30
//#define BLUE_ON_TIME    4
//#define BLUE_OFF_TIME   4
//#define WHITE_ON_TIME   5
//#define WHITE_OFF_TIME  5



#define RED_ON_TIME     20
#define RED_OFF_TIME    5
#define GREEN_ON_TIME   1
#define GREEN_OFF_TIME  1
#define BLUE_ON_TIME    2
#define BLUE_OFF_TIME  	2
#define WHITE_ON_TIME   2
#define WHITE_OFF_TIME  2


#define TEMP_TH         30.0 // 기�? ?��?��

#define RED_ON		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
#define GREEN_ON	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
#define BLUE_ON		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
#define WHITE_ON	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

#define RED_OFF		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
#define GREEN_OFF	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
#define BLUE_OFF	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
#define WHITE_OFF	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
#define LED_PORT GPIOB

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_WHITE_Pin GPIO_PIN_12
#define LED_WHITE_GPIO_Port GPIOB
#define LED_BLUE_Pin GPIO_PIN_13
#define LED_BLUE_GPIO_Port GPIOB
#define LED_GREEN_Pin GPIO_PIN_14
#define LED_GREEN_GPIO_Port GPIOB
#define LED_RED_Pin GPIO_PIN_15
#define LED_RED_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
void Led_toggle_config(LED_T *led,uint32_t on_time, uint32_t off_time,uint16_t pin);
void Led_toggle_config2(LED_T *led,uint32_t on_time, uint32_t off_time,uint16_t pin);

float Check_temperture();
void Green_config();
void Blue_White_config();

uint8_t Check_button();
uint8_t Init_All_Led();
uint32_t Get_Now_RTC_time();

void time_to_sec(TIME_T time, uint32_t wait_time,TIME_T *dest);
void time_to_sec_config();

uint8_t Get_time(LED_T *led);

void Read_RTC_TR(TIME_T* time);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
