/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "rtc.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <math.h>

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

/* USER CODE BEGIN PV */
LED_T m_red;
LED_T m_green;
LED_T m_blue;
LED_T m_white;

uint8_t bw_step = 0;



extern RTC_HandleTypeDef hrtc;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)///qw
{
  /* USER CODE BEGIN 1 */
  TIME_T time = {0,};

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
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    switch(m_red.step)
 	{
		case STEP1:
			RED_ON;
			//m_red.past_time = Get_Now_RTC_time();

			Read_RTC_TR(&time);
			time_to_sec(time,RED_ON_TIME, &m_red.dest_time);
			m_red.step = STEP2;
		break;

		case STEP2:
			Green_config(); 	
			Blue_White_config();
	
		    //if(Get_Now_RTC_time() >= m_red.past_time + RED_ON_TIME)
		    if(Get_time(&m_red))
	 		{ 			
				Init_All_Led();
				//m_red.past_time = Get_Now_RTC_time();
				Read_RTC_TR(&time);
				time_to_sec(time,RED_OFF_TIME, &m_red.dest_time);
				m_red.step = STEP3;
	 		}	
		break;

		case STEP3:
			//if(Get_Now_RTC_time() >= m_red.past_time + RED_OFF_TIME)
			if(Get_time(&m_red))
	 		{
				m_red.step = STEP1;
	 		}	
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_HIGH);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void Led_toggle_config(LED_T *led,uint32_t on_time, uint32_t off_time,uint16_t pin)
{
	TIME_T dest_time = {0,};

	RTC_TimeTypeDef RTC_times;
	switch(led->step)
	{
		case STEP1:
			HAL_GPIO_WritePin(LED_PORT, pin, GPIO_PIN_SET); 
			led->step = STEP2;
			led->on_flag = ON;
			led->past_time = Get_Now_RTC_time();
		break;

		case STEP2:
			if(Get_Now_RTC_time() >= led->past_time + on_time )
	 		{
				led->step = STEP3;
	 		}
		break;

		case STEP3:
			HAL_GPIO_WritePin(LED_PORT, pin, GPIO_PIN_RESET); 
			led->step = STEP4;
			led->on_flag = OFF;
			led->past_time = Get_Now_RTC_time();

		break;

		case STEP4:
			if(Get_Now_RTC_time() >= led->past_time + off_time )
	 		{
				led->step = STEP1;
	 		}
		break;	
	}

}


void Led_toggle_config2(LED_T *led,uint32_t on_time, uint32_t off_time,uint16_t pin)
{
	TIME_T time = {0,};
	switch(led->step)
	{
		case STEP1:
			HAL_GPIO_WritePin(LED_PORT, pin, GPIO_PIN_SET); 
			led->step = STEP2;
			
			Read_RTC_TR(&time);
			time_to_sec(time,on_time, &led->dest_time);

		break;

		case STEP2:
			if(Get_time(led))
	 		{
				led->step = STEP3;
	 		}
		break;

		case STEP3:
			HAL_GPIO_WritePin(LED_PORT, pin, GPIO_PIN_RESET); 
			led->step = STEP4;
			
			Read_RTC_TR(&time);
			time_to_sec(time,off_time,&led->dest_time);
		break;

		case STEP4:
			if(Get_time(led))
	 		{
				led->step = STEP1;
	 		}
		break;	
	}

}

uint32_t temp = 0;
float Check_temperture()
{
	uint32_t adc_value;
	float Vin,R1 = 0;
	float a,b = 0;

	
	adc_value = Read_ADC();

	if(adc_value==0) return temp;
	
 	Vin = (adc_value / 4095.0) * 5.0;
 	R1 = Vin/(5.0 - Vin) * 10000.0; 
 	
	 if(R1 > 26250.0)
	 {
	   a = -20.0;
	   b = 208.47;
	 }
	 else if(R1 > 4025.0)
	 {
	   a = -26.6;
	   b = 270.54;
	 }
	 else
	 {
	   a = -36.18;
	   b = 349.45;
	 }
	 
	 temp =a*log(R1) + b;
	 
	 return temp;
}

void Green_config()
{
	uint32_t temper = 0;
	switch(m_green.sub_step)
	{
		case STEP1:
			Led_toggle_config2(&m_green, GREEN_ON_TIME, GREEN_OFF_TIME, LED_GREEN_Pin);
			
			
			temper = Check_temperture();			
			if(temper >= TEMP_TH) 
			{
				memset(&m_green,0,sizeof(LED_T));
				GREEN_ON;
				m_green.sub_step = STEP2; 
			}
		break;
		case STEP2:
			
			temper = Check_temperture();			
			if(temper < TEMP_TH) 
			{
				GREEN_OFF;
				m_green.sub_step = STEP1; 
			}
		break;

		
	}
	
}

void Blue_White_config()
{
	
	switch(bw_step)
	{
		case STEP1:
			if(Check_button() == ON)
			{
				memset(&m_blue,0,sizeof(LED_T));
				BLUE_OFF;
				bw_step = STEP2; 
			}
			Led_toggle_config2(&m_blue, BLUE_ON_TIME, BLUE_OFF_TIME, LED_BLUE_Pin);	
			
		break;
		case STEP2:
			if(Check_button() == OFF) 
			{
				memset(&m_white,0,sizeof(LED_T));
				WHITE_OFF;
				bw_step = STEP1; 
			}	
			Led_toggle_config2(&m_white, WHITE_ON_TIME, WHITE_OFF_TIME, LED_WHITE_Pin); 			
		break;

		
	}

}

uint8_t Check_button()
{
	GPIO_PinState button0,button1 = 0;
	
	button0 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
	button1 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);

	if(button0 && button1)
	{
		return ON;	
	}
	else
	{
		return OFF;
	}
}

uint8_t Init_All_Led()
{
	memset(&m_red,0,sizeof(LED_T));
	memset(&m_blue,0,sizeof(LED_T));
	memset(&m_green,0,sizeof(LED_T));
	memset(&m_white,0,sizeof(LED_T));
	bw_step = 0;

	RED_OFF;	GREEN_OFF;	BLUE_OFF;	WHITE_OFF;
}

uint32_t Get_Now_RTC_time()
{
	uint32_t time = 0;

	time = HAL_GetTick();

	return time;
	//return (RTC->CNTH<< 16 | RTC->CNTL);
}
RTC_TimeTypeDef times;

uint8_t Get_time(LED_T *led)
{
	TIME_T time = {0,};

	Read_RTC_TR(&time);

	if(led->dest_time.hour <= time.hour && led->dest_time.min <= time.min && led->dest_time.sec <= time.sec )
	{
		return SET;
	}
	return RESET;
}

void Read_RTC_TR(TIME_T* time)
{
	volatile uint32_t tr = 0;

	tr = RTC->TR;

	time->sec = ((tr>>4) & 0x07)*10 + (tr&0x0f);//((tr>>4) & 0x07)*10 + tr&0x0f;
	time->min = ((tr>>12) & 0x07)*10 + ((tr>>8)&0x0f);
	time->hour = ((tr>>20) & 0x07)*10 + ((tr>>16)&0x0f);
}

uint32_t now_sec = 0;
void time_to_sec(TIME_T time, uint32_t wait_time,TIME_T *dest)
{

	TIME_T add;

	add.hour = wait_time/3600;
	add.min = (wait_time%3600)/60;
	add.sec = wait_time%60;
	
	dest->hour = time.hour + add.hour;
	dest->min = time.min +add.min;
	dest->sec = time.sec + add.sec;

	if(dest->sec >= 60)
	{
		dest->sec -= 60;
		dest->min++;
	}
	
	if(dest->min >= 60)
	{
		dest->min -= 60;
		dest->hour++;
	}
	
	if(dest->hour > 12)
	{
		dest->hour -= 12;
	}

}
void time_to_sec_config()
{
	TIME_T dest_time = {0,};
	
	HAL_RTC_GetTime(&hrtc, &times,RTC_FORMAT_BIN);
	HAL_Delay(10);
	//time_to_sec(&times,5000,&dest_time);
	
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
