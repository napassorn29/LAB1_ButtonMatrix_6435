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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

typedef struct _CheckState
{
	uint16_t current;
	uint16_t last;
}CheckState;

CheckState Check[16] =
{
	{1, 1},
	{1, 1},
	{1, 1},
	{1, 1},
	{1, 1},
	{1, 1},
	{1, 1},
	{1, 1},
	{1, 1},
	{1, 1},
	{1, 1},
	{1, 1},
	{1, 1},
	{1, 1},
	{1, 1},
	{1, 1}
}; // Check State

typedef struct _PortPin
{
 GPIO_TypeDef* PORT;
 uint16_t PIN;
}PortPin;

PortPin R[4] =
{
  {GPIOA,GPIO_PIN_10},
  {GPIOB,GPIO_PIN_3},
  {GPIOB,GPIO_PIN_5},
  {GPIOB,GPIO_PIN_4}

};

PortPin L[4] =
{
  {GPIOA,GPIO_PIN_9},
  {GPIOC,GPIO_PIN_7},
  {GPIOB,GPIO_PIN_6},
  {GPIOA,GPIO_PIN_7}

};

uint16_t ButtonMatrix = 0;
uint16_t Test = 0;
uint16_t edge = 0;

int num[11]={0}, j = 0;
int StudentNumber[11] = {6,4,3,4,0,5,0,0,0,3,5};
int checkLED = 0;
int SameNum = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void StateNum();
void ReadMatrixButton_1Row();
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // call  function every 10 ms == 100 Hz
	  static uint32_t timestamp = 0;
	     if(HAL_GetTick()>=timestamp)
	     {
	      timestamp = HAL_GetTick() + 10;
	      ReadMatrixButton_1Row();
	     }
	     StateNum();

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA7 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

//Read matrix button
void ReadMatrixButton_1Row()
	{
	static uint8_t X = 0;
	// read L1-L4
	register int i;
	for(i=0;i<4;i++)
	{
		if(HAL_GPIO_ReadPin(L[i].PORT, L[i].PIN)) //== 1(high)
		{
			Check[X*4+i].current = 1;
			ButtonMatrix &= ~(1<<(X*4+i));
			// i == 0, x == 0
			// ~(1<<0)
			// ~1
			// ~0b0000000000000001
			// 0b1111111111111110
		}
		else
		{
			Check[X*4+i].current = 0;
			ButtonMatrix |= 1<<(X*4+i);
			//B1.current = ButtonMatrix |= 1<<(X*4+i);
			// 0b0000000000000100

			if(Check[X*4+i].last == 1 && Check[X*4+i].current == 0)
			{
				edge = 1;
			}
		}

		Check[X*4+i].last = Check[X*4+i].current;
	}
	HAL_GPIO_WritePin(R[X].PORT, R[X].PIN, 1);
	HAL_GPIO_WritePin(R[(X+1)%4].PORT, R[(X+1)%4].PIN, 0);
	X++;
	X%=4;
	}


void StateNum()
{
    if (ButtonMatrix == 4096 && edge == 1)
    {
    	 j = 0;
    	 for(int m=0;m<12;m++)
    	 {
    		 num[m]=0;
    	 }
    	 HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
    }
    else if (ButtonMatrix == 8192 && edge == 1)
    {
    	 num[j-1] = 0;
    	 j = j-1;
    	 checkLED = 0;
    	 HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
    }
    else if(ButtonMatrix == 8 && edge == 1)
    {
    	 num[j] = 0;
   	 	 j=j+1;
   	 	 checkLED = 0;
   	 	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
    }
    else if(ButtonMatrix == 4 && edge == 1)
    {
    	 num[j] = 1;
   	 	 j=j+1;
   	 	 checkLED = 0;
   	 	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
    }
    else if(ButtonMatrix == 64 && edge == 1)
    {
    	 num[j] = 2;
    	 j=j+1;
    	 checkLED = 0;
    	 HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
    }
    else if(ButtonMatrix == 1024 && edge == 1)
    {
    	 num[j] = 3;
    	 j=j+1;
    	 checkLED = 0;
    	 HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
    }
    else if(ButtonMatrix == 2 && edge == 1)
    {
    	 num[j] = 4;
    	 j=j+1;
    	 checkLED = 0;
    	 HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
    }
    else if(ButtonMatrix == 32 && edge == 1)
    {
    	 num[j] = 5;
    	 j=j+1;
    	 checkLED = 0;
    	 HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
    }
    else if(ButtonMatrix == 512 && edge == 1)
    {
    	 num[j] = 6;
    	 j=j+1;
    	 checkLED = 0;
    	 HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
    }
    else if(ButtonMatrix == 1 && edge == 1)
    {
    	 num[j] = 7;
    	 j=j+1;
    	 checkLED = 0;
    	 HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
    }
    else if(ButtonMatrix == 16 && edge == 1)
    {
    	 num[j] = 8;
    	 j=j+1;
    	 checkLED = 0;
    	 HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
    }
    else if(ButtonMatrix == 256 && edge == 1)
    {
    	 num[j] = 9;
    	 j=j+1;
    	 checkLED = 0;
    	 HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
    }
    else if(ButtonMatrix == 32768 && edge == 1)
    {
    	 for(int k=0;k<12;k++)
    	 {
    		 if(num[k] == StudentNumber[k])
    		 {
    			 SameNum = SameNum + 1;
    		 }
    	 }
    	 if(SameNum == 11)
    	 {
    		 checkLED = 1;
    		 HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
    		 j = 0;
    		 for(int m=0;m<21;m++)
    		 {
    			 num[m]=0;
    		 }
    	 }
    	 SameNum = 0;
    }

    edge = 0;
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
