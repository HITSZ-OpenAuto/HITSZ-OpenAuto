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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32_2.8_lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define GPIO_OUT1_ON HAL_GPIO_WritePin(UL_GPIO_Port,UL_Pin,GPIO_PIN_SET)
#define GPIO_OUT2_ON HAL_GPIO_WritePin(VL_GPIO_Port,VL_Pin,GPIO_PIN_SET)
#define GPIO_OUT3_ON HAL_GPIO_WritePin(WL_GPIO_Port,WL_Pin,GPIO_PIN_SET)

#define GPIO_OUT1_OFF HAL_GPIO_WritePin(UL_GPIO_Port,UL_Pin,GPIO_PIN_RESET)
#define GPIO_OUT2_OFF HAL_GPIO_WritePin(VL_GPIO_Port,VL_Pin,GPIO_PIN_RESET)
#define GPIO_OUT3_OFF HAL_GPIO_WritePin(WL_GPIO_Port,WL_Pin,GPIO_PIN_RESET)

#define TIM8_CH1_ON HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1)
#define TIM8_CH2_ON HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2)
#define TIM8_CH3_ON HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3)

#define TIM8_CH1_OFF HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_1)
#define TIM8_CH2_OFF HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_2)
#define TIM8_CH3_OFF HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_3)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

int cPeriod = 0;
int hall_a = 0;
int hall_b = 0;
int hall_c = 0;
int mode = 0;
float speed = 0;
float angle = 0;
int sel_mode = 0;

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
  MX_ADC1_Init();
  MX_ADC3_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	__HAL_TIM_ENABLE_IT(&htim8, TIM_IT_BREAK);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3);
	TIM8_CH1_ON;
	TIM8_CH2_ON;
	TIM8_CH3_ON;
	

	STM32_LCD_Init();
	LCD_Clear(BackColor);
	LCD_SetTextColor(Blue);
	LCD_DisplayStringLine(1,(u8 *)"BLDC off:");
	
	uint32_t vBuf[1] = {0};
	uint32_t pwmBuf[1] = {0};
	float voltage = 0;
	
	HAL_ADC_Start_DMA(&hadc3, pwmBuf, 1);
	HAL_ADC_Start_DMA(&hadc1, vBuf, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(sel_mode){
				if(mode == 0)
				{
					HAL_TIM_IC_CaptureCallback(&htim2);
					HAL_Delay(4);
					HAL_TIM_IC_CaptureCallback(&htim2);
					mode = 1;
				}
				
				HAL_ADC_Start_DMA(&hadc3, pwmBuf, 1);
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmBuf[0]);
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmBuf[0]);
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, pwmBuf[0]);

				HAL_Delay(50);
	
				HAL_ADC_Start_DMA(&hadc1, vBuf, 1);
				voltage = (int)vBuf[0]/7.32;
				
				HAL_Delay(100);
				
				LCD_SetTextColor(Blue);
				LCD_ClearLine(1);
				LCD_DisplayStringLine(2,(u8 *)"BLDC speed:");
				LCD_Draw_NUM(70,300,(int)speed);

				LCD_DisplayStringLine(5,(u8 *)"PWM:");
				LCD_Draw_NUM(70+70,300,(int)pwmBuf[0]);
				LCD_Draw_NUM(0,300,(int)pwmBuf[0]/4095.0*100);
				LCD_DisplayStringLine(7,(u8 *)"Voltage:");
				LCD_Draw_NUM(70+70+70,300,voltage);

		}
		else
		{
			LCD_ClearLine(1);
			LCD_DisplayStringLine(1,(u8 *)"BLDC off:");
			TIM8_CH1_OFF;
			TIM8_CH2_OFF;
			TIM8_CH3_OFF;
			GPIO_OUT1_OFF;
			GPIO_OUT2_OFF;
			GPIO_OUT3_OFF;
			mode = 0;
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(TIM3==htim->Instance)
	cPeriod++;
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (!sel_mode){
		return;
	}
	else{
		if(TIM2==htim->Instance)
		{
			if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15)==GPIO_PIN_SET) hall_a = 1;
			else hall_a = 0;
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_3)==GPIO_PIN_SET) hall_b = 1;
			else hall_b = 0;
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_10)==GPIO_PIN_SET) hall_c = 1;
			else hall_c = 0;
			
			if(hall_a==1 && hall_b==0 && hall_c==1)
			{
				TIM8_CH1_ON;
				TIM8_CH2_OFF;
				TIM8_CH3_OFF;
				GPIO_OUT1_OFF;
				GPIO_OUT2_ON;
				GPIO_OUT3_OFF;
			}
			else if(hall_a==1 && hall_b==0 && hall_c==0)
			{
				TIM8_CH1_ON;
				TIM8_CH2_OFF;
				TIM8_CH3_OFF;
				GPIO_OUT1_OFF;
				GPIO_OUT2_OFF;
				GPIO_OUT3_ON;
			}
			else if(hall_a==1 && hall_b==1 && hall_c==0)
			{
				TIM8_CH1_OFF;
				TIM8_CH2_ON;
				TIM8_CH3_OFF;
				GPIO_OUT1_OFF;
				GPIO_OUT2_OFF;
				GPIO_OUT3_ON;
			}
			else if(hall_a==0 && hall_b==1 && hall_c==0)
			{
				TIM8_CH1_OFF;
				TIM8_CH2_ON;
				TIM8_CH3_OFF;
				GPIO_OUT1_ON;
				GPIO_OUT2_OFF;
				GPIO_OUT3_OFF;
			}
			else if(hall_a==0 && hall_b==1 && hall_c==1)
			{
				TIM8_CH1_OFF;
				TIM8_CH2_OFF;
				TIM8_CH3_ON;
				GPIO_OUT1_ON;
				GPIO_OUT2_OFF;
				GPIO_OUT3_OFF;
			}
			else if(hall_a==0 && hall_b==0 && hall_c==1)
			{
				TIM8_CH1_OFF;
				TIM8_CH2_OFF;
				TIM8_CH3_ON;
				GPIO_OUT1_OFF;
				GPIO_OUT2_ON;
				GPIO_OUT3_OFF;
			}
			else{
				return;
			}
			angle += 60;
			if(cPeriod >= 50){
				speed = angle/(cPeriod*0.001)/360/3;
				cPeriod = 0;
				angle = 0;
			}

		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	int i=0;
	if(HAL_GPIO_ReadPin(KEY_SEL_GPIO_Port,KEY_SEL_Pin)==GPIO_PIN_RESET)
			{
		for(i=0;i<10000;i++){}
				if(HAL_GPIO_ReadPin(KEY_SEL_GPIO_Port,KEY_SEL_Pin)==GPIO_PIN_RESET){
					sel_mode = 1;
					return;
				}
			}
	sel_mode = 0;

}

void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef* htim)
{
	if(htim->Instance==TIM8)
	{
		TIM8_CH1_OFF;
		TIM8_CH2_OFF;
		TIM8_CH3_OFF;
		GPIO_OUT1_OFF;
		GPIO_OUT2_OFF;
		GPIO_OUT3_OFF;
		__HAL_TIM_DISABLE_IT(&htim8, TIM_IT_BREAK);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
