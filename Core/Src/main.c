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
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
struct __FILE
{
  int handle;
  /* Whatever you require here. If the only file you are using is */
  /* standard output using printf() for debugging, no file handling */
  /* is required. */
};
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define csOn()	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET)
#define csOf()	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET)

uint8_t cbusReadData8bit(uint8_t reg)
{
	uint8_t data;
	csOn();
	HAL_SPI_Transmit(&hspi1, &reg, 1, 1000);
	HAL_SPI_Receive(&hspi1, &data, 1, 1000);
	csOf();
	return data;
}
uint16_t cbusReadData16bit(uint8_t reg)
{
	uint8_t data;
	uint16_t data16;
	csOn();
	HAL_SPI_Transmit(&hspi1, &reg, 1, 1000);
	HAL_SPI_Receive(&hspi1, &data, 1, 1000);
	data16 = data<<8;
	HAL_SPI_Receive(&hspi1, &data, 1, 1000);
	data16 = data16|data;
	csOf();
	return data16;
}	
uint8_t cbusReadData8bit_save ( uint8_t reg , uint8_t *data)
{
	csOn();
	HAL_SPI_Transmit(&hspi1, &reg , 1, 1000);
	
	HAL_SPI_Receive(&hspi1, &data[0] , 27 , 1000);
	
	csOf();
}
void cbusWriteData8bit_save(uint8_t reg, uint8_t *data)
{
	csOn();
	HAL_SPI_Transmit(&hspi1, &reg, 1, 1000);
	HAL_SPI_Transmit(&hspi1, &data[0], 1, 1000);
	csOf();
}

void cbusWriteData8bit(uint8_t reg, uint8_t data)
{
	csOn();
	HAL_SPI_Transmit(&hspi1, &reg, 1, 1000);
	HAL_SPI_Transmit(&hspi1, &data, 1, 1000);
	csOf();
}
void cbusWriteData16bit(uint8_t reg, uint16_t data16)
{
	uint8_t data[2];
	data[0]=data16>>8;
	data[1]=data16&0xff;
	csOn();
	HAL_SPI_Transmit(&hspi1, &reg, 1, 1000);
	HAL_SPI_Transmit(&hspi1, &data[0], 2, 1000);
	csOf();
}	
uint8_t dataBuff[2700] = {0};
int idx =0;
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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
	cbusWriteData8bit(0x01, 1);
	while(1)
	{
			uint16_t data16 = cbusReadData16bit(0x40);
			if(data16&0x8000)
			{
				printf("\n RDY Data16= 0x%04x", data16);
				break;
			}
			else	printf("\n Fail: Data16= 0x%04x", data16);
					
	}	
	cbusWriteData16bit(0x1f, 0xc107);
	cbusWriteData16bit(0x1d, 0x0005);
	cbusWriteData8bit(0x0A, 0x00);
	while(1)
	{
			uint16_t data16 = cbusReadData16bit(0x40);
			if(data16&0x4000)
			{
				printf("\n SVC Data16= 0x%04x", data16);
				break;
			}
			else	printf("\n Fail: Data16= 0x%04x", data16);
					
	}	
	
	cbusWriteData8bit(0x09, 0x03);
	HAL_Delay(100);
	cbusWriteData8bit(0x07, 0x37);
	while(1)
	{
			uint16_t data16 = cbusReadData16bit(0x40);
			if(data16&0x8000)
			{
				printf("\n RDY Data16= 0x%04x", data16);
				break;
			}
			else	printf("\n Fail: Data16= 0x%04x", data16);
					
	}	
	while(1)
	{
			uint16_t data = cbusReadData8bit(0x2E);
			if(data&0x01)
			{
				printf("\n 2E Data= 0x%02x", data);
				break;
			}
			else	printf("\n Fail: Data= 0x%02x", data);
					
	}	
	HAL_Delay(100);
	
	cbusWriteData16bit(0x11, 0x0002);
	while(1)
	{
			uint16_t data16 = cbusReadData16bit(0x40);
			if(data16&0x8000)
			{
				printf("\n RDY Data16= 0x%04x", data16);
				break;
			}
			else	printf("\n Fail: Data16= 0x%04x", data16);
					
	}	
	while(1)
	{
			uint16_t data = cbusReadData8bit(0x2E);
			if(data&0x01)
			{
				printf("\n 2E Data= 0x%02x", data);
				break;
			}
			else	printf("\n Fail: Data= 0x%02x", data);
					
	}		
	idx =0;
	while(1)
	{			
			uint16_t data16 = cbusReadData16bit(0x40);
			if(data16&0x0001){
				cbusReadData8bit_save (0x30,&dataBuff[idx]);
				idx += 27; if(idx >= 2700) break;
			}
	}	
	for( int i = 0 ; i < 2700 ; i ++){
		printf("0x%02x,",dataBuff[i]);
	}
	cbusWriteData16bit(0x11,0x0002);
	while(1)
	{
			uint16_t data16 = cbusReadData16bit(0x40);
			if(data16&0x8000)
			{
				printf("\n RDY_1 Data16= 0x%04x", data16);
				break;
			}
			else	printf("\n Fail: Data16= 0x%04x", data16);
					
	}	
	while(1)
	{
			uint16_t data = cbusReadData8bit(0x2E);
			if(data&0x01)
			{
				printf("\n 2E_1s Data= 0x%02x", data);
				break;
			}
			else	printf("\n Fail: Data= 0x%02x", data);
					
	}		
while(1)
	{			
				cbusWriteData8bit_save (0x10,&dataBuff[idx]);
				idx += 27;
			HAL_Delay(60);
		if(idx >= 2700) break;
	}	
		for( int i = 0 ; i < 2700 ; i ++){
		printf("\n 0x%02x,",dataBuff[i]);
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
int fputc(int ch, FILE *f) {
    ITM_SendChar(ch);//send method for SWV
    return(ch);
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
