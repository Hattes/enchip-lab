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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
int is_blue_button_pressed();
void put_die_dots(uint8_t);
void reset_die();
void put_on_sseg(uint8_t dec_nbr);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

const uint8_t sseg[10] = {
		0x3f,  // 0
		0x06,  // 1
		0x5b,  // 2
		0x4f,  // 3
		0x66,  // 4
		0x6d,  // 5
		0x7d,  // 6
		0x07,  // 7
		0x7f,  // 8
		0x6f,  // 9
		//0xdc   // err
};
const uint8_t sseg_err = 0xdc;

void put_on_sseg(uint8_t dec_nbr) {
	uint8_t pattern;
	if (dec_nbr >= 0 && dec_nbr <= 9) {
		pattern = sseg[dec_nbr];
	}
	else {
		pattern = sseg_err;
	}
	GPIOC->ODR = ~pattern;
}

int is_blue_button_pressed()
{
	uint32_t reg_reading = GPIOC->IDR;
	// Should be bit 13
	// to get a bit, shift right by position number and '&' (bitwise and) with 0x01
	return !((reg_reading >> 13) & 0x01);
}

void put_die_dots(uint8_t die_value)
{
	GPIO_PinState a, b, c, d, e, f, g;
	a = b = c = d = e = f = g = GPIO_PIN_RESET;
	switch (die_value)
	{
	case 1:
		d = GPIO_PIN_SET;
		break;
	case 2:
		a = g = GPIO_PIN_SET;
		break;
	case 3:
		a = d = g = GPIO_PIN_SET;
		break;
	case 4:
		a = c = e = g = GPIO_PIN_SET;
		break;
	case 5:
		a = c = d = e = g = GPIO_PIN_SET;
		break;
	case 6:
		a = b = c = e = f = g = GPIO_PIN_SET;
		break;
	default:
		a = b = c = d = e = f = g = GPIO_PIN_SET;
	}
	HAL_GPIO_WritePin(DI_A_GPIO_Port, DI_A_Pin, a);
	HAL_GPIO_WritePin(DI_B_GPIO_Port, DI_B_Pin, b);
	HAL_GPIO_WritePin(DI_C_GPIO_Port, DI_C_Pin, c);
	HAL_GPIO_WritePin(DI_D_GPIO_Port, DI_D_Pin, d);
	HAL_GPIO_WritePin(DI_E_GPIO_Port, DI_E_Pin, e);
	HAL_GPIO_WritePin(DI_F_GPIO_Port, DI_F_Pin, f);
	HAL_GPIO_WritePin(DI_G_GPIO_Port, DI_G_Pin, g);
}

void reset_die()
{
	HAL_GPIO_WritePin(DI_A_GPIO_Port, DI_A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DI_B_GPIO_Port, DI_B_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DI_C_GPIO_Port, DI_C_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DI_D_GPIO_Port, DI_D_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DI_E_GPIO_Port, DI_E_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DI_F_GPIO_Port, DI_F_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DI_G_GPIO_Port, DI_G_Pin, GPIO_PIN_RESET);
}

uint32_t apa[10];

void test_endianness()
{
	// Initialize the array
	for (int i = 0; i < 10; i+= 2)
	{
		apa[i] = 0xDEADBEEF;
		apa[i+1] = 0xCAFED00D;
	}

	uint16_t a = apa[0];
	uint16_t b = apa[1];
	uint16_t c = apa[2];

	uint8_t x = apa[0];
	uint8_t y = apa[1];

	uint8_t *apa8 = (uint8_t *) apa;

	uint8_t p = apa8[0];
	uint8_t q = apa8[1];
	uint8_t r = apa8[2];
	uint8_t s = apa8[3];
	uint8_t t = apa8[4];

	uint16_t babe = 0xBABE;
	apa[0] = babe;

	return;
}

int pressed = 0;
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
  //test_endianness();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t die_value = 0; // 1 <= die_value <= 6
  while (1)
  {
	  if (is_blue_button_pressed())
	  {
		  reset_die();
		  //put_on_sseg(0x00E);
		  // inc die
		  die_value++;
		  if (die_value > 6) {
			  die_value = 1;
		  }
		  HAL_Delay(1);

		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	  }
	  else {
		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	  }

	  put_die_dots(die_value);
	  //put_on_sseg(die_value);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|DI_D_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|DI_A_Pin|DI_B_Pin|DI_F_Pin
                          |DI_E_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DI_G_Pin|DI_C_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3
                           PC4 PC5 PC6 DI_D_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|DI_D_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin DI_A_Pin DI_B_Pin DI_F_Pin
                           DI_E_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|DI_A_Pin|DI_B_Pin|DI_F_Pin
                          |DI_E_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DI_G_Pin DI_C_Pin */
  GPIO_InitStruct.Pin = DI_G_Pin|DI_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
