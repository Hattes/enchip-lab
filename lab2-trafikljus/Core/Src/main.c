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
#include <stdio.h>
#include "abuzz.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// TODO add missing all-red states
enum TrafficLightState
{
	s_init,
	s_all_stop,
	s_cars_go,
	s_button_pressed,
	s_cars_stop,
	s_pedestrians_go,
	s_cars_start,
	s_NUM_STATES,
};

enum TrafficEvent
{
	ev_none,
	ev_button_press,
	ev_state_timeout,
	ev_error = -99,
};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USING_ISR 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
int is_blue_button_pressed();
void set_traffic_lights(enum TrafficLightState tls);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int is_blue_button_pressed()
{
	uint32_t reg_reading = GPIOC->IDR;
	// Should be bit 13
	// to get a bit, shift right by position number and '&' (bitwise and) with 0x01
	return !((reg_reading >> 13) & 0x01);
}

void set_traffic_lights(enum TrafficLightState tls)
{
	GPIO_PinState cr, cy, cg, pr, pg;
	cr = cy = cg = pr = pg = GPIO_PIN_RESET;
	switch (tls)
	{
	case s_init:
		cr = cy = cg = pr = pg = GPIO_PIN_SET;
		break;
	case s_all_stop:
		cr = pr = GPIO_PIN_SET;
		break;
	case s_cars_go:
	case s_button_pressed:
		cg = pr = GPIO_PIN_SET;
		break;
	case s_cars_stop:
		cy = pr = GPIO_PIN_SET;
		break;
	case s_pedestrians_go:
		cr = pg = GPIO_PIN_SET;
		break;
	case s_cars_start:
		cy = pr = GPIO_PIN_SET;
		break;
	}
	HAL_GPIO_WritePin(CAR_RED_GPIO_Port, CAR_RED_Pin, cr);
	HAL_GPIO_WritePin(CAR_YLW_GPIO_Port, CAR_YLW_Pin, cy);
	HAL_GPIO_WritePin(CAR_GRN_GPIO_Port, CAR_GRN_Pin, cg);
	HAL_GPIO_WritePin(PED_RED_GPIO_Port, PED_RED_Pin, pr);
	HAL_GPIO_WritePin(PED_GRN_GPIO_Port, PED_GRN_Pin, pg);
	return;
}

#define EVQ_SIZE 10

enum TrafficEvent evq[ EVQ_SIZE ];
int evq_count		= 0;
int evq_front_ix 	= 0;
int evq_rear_ix		= 0;

void evq_init()
{
	for (int x = 0; x < EVQ_SIZE; x++)
	{
		evq[x] = ev_error;
	}
}

void evq_push_back(enum TrafficEvent e)
{

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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	abuzz_start();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	enum TrafficLightState state = s_init;
	enum TrafficEvent ev = ev_none;
	int curr_pressed, last_pressed;
	curr_pressed = last_pressed = is_blue_button_pressed();
	uint32_t curr_tick, last_tick;
	curr_tick = last_tick = HAL_GetTick();
	int32_t ticks_left_in_state = 0;

	set_traffic_lights(state);

	while (1) {

#if USING_ISR
#else

		last_pressed = curr_pressed;
		curr_pressed = is_blue_button_pressed();


		if (curr_pressed && !last_pressed)
		{
			ev = ev_button_press;
		}
		else if (ticks_left_in_state > 0)
		{
			last_tick = curr_tick;
			curr_tick = HAL_GetTick();
			uint32_t delta = curr_tick - last_tick;
			if (delta != 0)
			{
				ticks_left_in_state -= 1;
				if (ticks_left_in_state == 0)
				{
					ev = ev_state_timeout;
				}
				else
				{
					ev = ev_none;
				}
			}
		}
		else
		{
			ev = ev_none;
		}
#endif

		switch(state)
		{
		case s_init:
			if (ev == ev_button_press) {
				state = s_all_stop;
				ticks_left_in_state = 2000;
				ev = ev_none;
				set_traffic_lights(s_all_stop);
				abuzz_p_long();
			}
			break;
		case s_all_stop:
			if (ev == ev_state_timeout) {
				state 				= s_cars_go;
				ticks_left_in_state = 0;
				ev 			= ev_none;
				set_traffic_lights(s_cars_go);
			}
			break;
		case s_cars_go:
			if (ev == ev_button_press) {
				state 				= s_button_pressed;
				ticks_left_in_state = 3000;
				ev 			= ev_none;
				set_traffic_lights(s_button_pressed);
			}
			break;
		case s_button_pressed:
			if (ev == ev_state_timeout) {
				state 				= s_cars_stop;
				ticks_left_in_state = 2500;
				ev 			= ev_none;
				set_traffic_lights(s_cars_stop);
			}
			break;
		case s_cars_stop:
			if (ev == ev_state_timeout) {
				state				= s_pedestrians_go;
				ticks_left_in_state = 5000;
				ev			= ev_none;
				set_traffic_lights(s_pedestrians_go);
				abuzz_p_short();
			}
			break;
		case s_pedestrians_go:
			if (ev == ev_state_timeout) {
				state 				= s_cars_start;
				ticks_left_in_state = 2500;
				ev 			= ev_none;
				set_traffic_lights(s_cars_start);
				abuzz_p_long();
			}
			break;
		case s_cars_start:
			if (ev == ev_state_timeout) {
				state 				= s_cars_go;
				ticks_left_in_state = 0;
				ev 			= ev_none;
				set_traffic_lights(s_cars_go);
			}
			break;
		}
	}
	//for (TrafficLightState s = s_init; s <= s_cars_start; s++) {
	//set_traffic_lights(s);
	//HAL_Delay(300);
	//}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CAR_RED_Pin|CAR_YLW_Pin|CAR_GRN_Pin|PED_RED_Pin
                          |PED_GRN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CAR_RED_Pin CAR_YLW_Pin CAR_GRN_Pin PED_RED_Pin
                           PED_GRN_Pin */
  GPIO_InitStruct.Pin = CAR_RED_Pin|CAR_YLW_Pin|CAR_GRN_Pin|PED_RED_Pin
                          |PED_GRN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
