/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
#include "LIS3MDL.h"

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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

#define REFRESH_DELAY 	10
#define SENSOR_NUM 			256

//float magValue[3];
//float temp;

uint8_t Charbuf[32] = {0};  //������
uint32_t Numbuf[8] = {0};
float debugbuf[3*4];

int16_t sendbuf[2000];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void HexToAscii(uint8_t *src, char *dest, int len);
void HexToNum(uint8_t *src, int len);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define 	WARNING_LED() 			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET)
#define		HC595_CLEAR_ON()		HAL_GPIO_WritePin(MR_GPIO_Port, MR_Pin, GPIO_PIN_RESET)
#define		HC595_CLEAR_OFF()		HAL_GPIO_WritePin(MR_GPIO_Port, MR_Pin, GPIO_PIN_SET)
#define		HC595_RCK_HIGH()		HAL_GPIO_WritePin(STCP_GPIO_Port, STCP_Pin, GPIO_PIN_SET)
#define		HC595_RCK_LOW()			HAL_GPIO_WritePin(STCP_GPIO_Port, STCP_Pin, GPIO_PIN_RESET)
#define		HC595_SCK_HIGH()		HAL_GPIO_WritePin(SHCP_GPIO_Port, SHCP_Pin, GPIO_PIN_SET)
#define		HC595_SCK_LOW()			HAL_GPIO_WritePin(SHCP_GPIO_Port, SHCP_Pin, GPIO_PIN_RESET)
#define		HC595_DS_HIGH()			HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, GPIO_PIN_SET)
#define		HC595_DS_LOW()			HAL_GPIO_WritePin(DS_GPIO_Port, DS_Pin, GPIO_PIN_RESET)

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	uint16_t p=0;
	LIS3MDL_t sensor;

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
  MX_I2C1_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	HC595_CLEAR_ON();
	HAL_Delay(1);
	HC595_CLEAR_OFF();
	HC595_DS_HIGH();
	HAL_Delay(1);
	for(uint16_t u=0; u<SENSOR_NUM; u++)
	{
		HC595_SCK_HIGH();  
		HC595_RCK_HIGH();
		HC595_DS_LOW();
		HC595_SCK_LOW(); 
		HC595_RCK_LOW();
//		HAL_Delay(1);
		
		if(LIS3MDL_Init(&sensor, &hi2c1, LIS3MDL_Device_1, LIS3MDL_Scale_16G, LIS3MDL_MODE_ULTRAHIGH, LIS3MDL_ODR_7) != HAL_OK)
				WARNING_LED();
		HAL_Delay(1);
	}
	
  while (1)
  {
		HAL_Delay(REFRESH_DELAY);
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);	
		
		HC595_CLEAR_ON();
		HC595_CLEAR_OFF();
		HC595_DS_HIGH();
		for(uint16_t u=0; u<SENSOR_NUM; u++)
		{
			HC595_SCK_HIGH();  
			HC595_RCK_HIGH();
			HC595_DS_LOW();
			HC595_SCK_LOW();  
			HC595_RCK_LOW();
			
			if(LIS3MDL_ReadMag(&sensor, &hi2c1) != HAL_OK) 
				WARNING_LED();
			p = u * 5;
			sendbuf[p+0] = 0xaaaa; sendbuf[p+1] = u; sendbuf[p+2] = sensor.mag_raw[0]; sendbuf[p+3] = sensor.mag_raw[1]; sendbuf[p+4] = sensor.mag_raw[2];
			
//			if(u==15)
//			{debugbuf[0]=sensor.mag[0]; debugbuf[1]=sensor.mag[1]; debugbuf[2]=sensor.mag[2];} 
//			else if(u==10)
//			{debugbuf[3]=sensor.mag[0]; debugbuf[4]=sensor.mag[1]; debugbuf[5]=sensor.mag[2];}
//			else if(u==14)
//			{debugbuf[6]=sensor.mag[0]; debugbuf[7]=sensor.mag[1]; debugbuf[8]=sensor.mag[2];}
//			else if(u==15)
//			{debugbuf[9]=sensor.mag[0]; debugbuf[10]=sensor.mag[1]; debugbuf[11]=sensor.mag[2];}
//			HAL_Delay(50);
			
			sensor.mag_raw[0] = 0; sensor.mag_raw[1] = 0; sensor.mag_raw[2] = 0;
			sensor.mag[0] = 0; sensor.mag[1] = 0; sensor.mag[2] = 0;
		}

		HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&sendbuf, SENSOR_NUM*5*2);  // sensornum * datalen(5) * 2. "*2": sendbuf is in uint16_t
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);	
			
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
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
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart1.Init.BaudRate = 256000;
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
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED2_Pin|LED1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DS_Pin|STCP_Pin|SHCP_Pin|MR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED2_Pin LED1_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DS_Pin STCP_Pin SHCP_Pin MR_Pin */
  GPIO_InitStruct.Pin = DS_Pin|STCP_Pin|SHCP_Pin|MR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


void HexToAscii(uint8_t *src, char *dest, int len)
{
	char dh,dl;  //?????????
	int i;
	for(i = 0; i < len; i++)
	{
		dh = '0' + src[i] / 16;
		dl = '0' + src[i] % 16;
		if(dh > '9')
		{
					 dh = dh - '9' - 1 + 'A'; // ?? dh= dh+ 7;
		}
		if(dl > '9')
		{
					 dl = dl - '9' - 1 + 'A'; // ??dl = dl + 7;
		}
		dest[2*i] = dh;
		dest[2*i+1] = dl;
	}
	dest[2*i] = '\0';
}

int converter(char src)
{
	switch(src)
	{
		case '0': return 0;
		case '1': return 1;
		case '2': return 2;
		case '3': return 3;
		case '4': return 4; 
		case '5': return 5;
		case '6': return 6;
		case '7': return 7;
		case '8': return 8;
		case '9': return 9;
		case 'A': return 10;
		case 'B': return 11;
		case 'C': return 12;
		case 'D': return 13;
		case 'E': return 14;
		case 'F': return 15;
		default: return 0;
	}
}
void HexToNum(uint8_t *src, int len)
{
	char dh,dl;
	char asc[32];
	int i;
	for(i = 0; i < len; i++)
	{
		dh = '0' + src[i] / 16;
		dl = '0' + src[i] % 16;
		if(dh > '9')
		{
			dh = dh - '9' - 1 + 'A'; // ?? dh= dh+ 7;
		}
		if(dl > '9')
		{
			dl = dl - '9' - 1 + 'A'; // ??dl = dl + 7;
		}
		asc[2*i] = dh;
		asc[2*i+1] = dl;
	}
	for(i = 0; i < len; i++)
	{
		Numbuf[i/2] = converter(asc[i*2])*16 + converter(asc[i*2+1]) + converter(asc[i*2+2])*16*16*16 + converter(asc[i*2+3])*16*16;
		i++;
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

