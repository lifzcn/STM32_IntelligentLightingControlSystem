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
#include "adc.h"
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "oled.h"
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
int main(void)
{
  /* USER CODE BEGIN 1 */
	RTC_DateTypeDef sdatestructure;
	RTC_TimeTypeDef stimestructure;
  uint8_t x = 0;
	uint8_t y = 0;
	uint8_t rxData[1];
	uint8_t markBit = 0;
	float numCmd;
	uint8_t dutyValue = 0;
	uint32_t ADC_Value[30];
	uint32_t ad1 = 0, ad2 = 0, ad3 = 0;
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
  MX_ADC1_Init();
  MX_RTC_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	OLED_Init();
	OLED_Clear();
	OLED_ShowChinese(x + 16 * 2, y, 0);
	OLED_ShowChinese(x + 16 * 3, y, 1);
	OLED_ShowChinese(x + 16 * 4, y, 2);
	OLED_ShowChinese(x + 16 * 5, y, 3);
	OLED_ShowChinese(x + 16 * 2, y + 2 * 1, 4);
	OLED_ShowChinese(x + 16 * 3, y + 2 * 1, 5);
	OLED_ShowChinese(x + 16 * 4, y + 2 * 1, 6);
	OLED_ShowChinese(x + 16 * 5, y + 2 * 1, 7);
	
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADC_Value, 30);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		HAL_UART_Receive(&huart1, rxData, 1, 0xffff);
		markBit = rxData[0] - (int)rxData[0];
		dutyValue = (int)rxData[0];
		switch(markBit)
		{
			case 1:
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, dutyValue);
			break;
			case 2:
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, dutyValue);
			break;
			case 3:
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, dutyValue);
			break;
			default:
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
			break;
		}

		HAL_RTC_GetTime(&hrtc, &stimestructure, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &sdatestructure, RTC_FORMAT_BIN);
		
		OLED_ShowNum(x + 24 + 8 * 0, y + 2 * 2, sdatestructure.Year + 2000, 4, 16);
		OLED_ShowChar(x + 24 + 8 * 4, y + 2 * 2, '-', 16);
		OLED_ShowNum(x + 24 + 8 * 5, y + 2 * 2, sdatestructure.Month, 2, 16);
		OLED_ShowChar(x + 24 + 8 * 7, y + 2 * 2, '-', 16);
		OLED_ShowNum(x + 24 + 8 * 8, y + 2 * 2, sdatestructure.Date, 2, 16);
		OLED_ShowNum(x + 32 + 8 * 0, y + 2 * 3, stimestructure.Hours, 2, 16);
		OLED_ShowChar(x + 32 + 8 * 2, y + 2 * 3, ':', 16);
		OLED_ShowNum(x + 32 + 8 * 3, y + 2 * 3, stimestructure.Minutes, 2, 16);
		OLED_ShowChar(x + 32 + 8 * 5, y + 2 * 3, ':', 16);
		OLED_ShowNum(x + 32 + 8 * 6, y + 2 * 3, stimestructure.Seconds, 2, 16);

		for(int i=0;i<30;)
		{
			ad1 += ADC_Value[i++];
			ad1 += ADC_Value[i++];
			ad1 += ADC_Value[i++];
		}
		printf("%d,%d,%d",ad1,ad2,ad3);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
