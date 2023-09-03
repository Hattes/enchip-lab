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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
	s_init,
	s_all_stop,
	s_cars_go,
	s_button_pressed,
	s_cars_stop,
	s_pedestrians_go,
	s_cars_start,
} TrafficLightState;
typedef enum
{
	ev_none,
	ev_button_press,
	ev_state_timeout
} event;
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
TrafficLightState state;
event last_event;
int ticks_left_in_state;
int button_pressed_prev_tick = 0;
uint32_t before;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
int is_blue_button_pressed();
void set_traffic_lights(TrafficLightState tls);
void tick();
event get_event(int check_button);
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

void set_traffic_lights(TrafficLightState tls)
{
	GPIO_PinState cr, cy, cg, pr, pg;
	cr = cy = cg = pr = pg = GPIO_PIN_RESET;
	switch (tls)
	{
	case s_init:
		cr = pr = GPIO_PIN_SET;
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
	HAL_GPIO_WritePin(GPIOC, CAR_RED_Pin, cr);
	HAL_GPIO_WritePin(GPIOC, CAR_YLW_Pin, cy);
	HAL_GPIO_WritePin(GPIOC, CAR_GRN_Pin, cg);
	HAL_GPIO_WritePin(GPIOC, PED_RED_Pin, pr);
	HAL_GPIO_WritePin(GPIOC, PED_GRN_Pin, pg);
	return;
}

void tick() {
	uint32_t now = HAL_GetTick();
	ticks_left_in_state -= now - before;
	before = now;
	return;
}

event get_event(int check_button) {
	if (check_button) {
		if (is_blue_button_pressed()) {
			if (!button_pressed_prev_tick) {
				button_pressed_prev_tick = 1;
				return ev_button_press;
			}
		} else {
			button_pressed_prev_tick = 0;
		}
	} else if (ticks_left_in_state <= 0) {
		return ev_state_timeout;
	}
	return ev_none;
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
  before = HAL_GetTick();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  state = s_init;
  set_traffic_lights(state);
  char str[81] = { '\0'};
  uint16_t str_len = 0;
  str_len = sprintf(str, "Test\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*) str, str_len, HAL_MAX_DELAY);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		tick();
		event new_event = get_event(state == s_cars_go);
		if (new_event == ev_none) {
			//str_len = sprintf(str, "Got none\r\n");
			//HAL_UART_Transmit(&huart2, (uint8_t*) str, str_len, HAL_MAX_DELAY);
		}
		if (new_event == ev_state_timeout) {
			str_len = sprintf(str, "Got timeout\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*) str, str_len, HAL_MAX_DELAY);
		}
		if (new_event == ev_button_press) {
			str_len = sprintf(str, "Got button press\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*) str, str_len, HAL_MAX_DELAY);
		}
		//str_len = sprintf(str, "Got event %d\r\n", new_event);
		//HAL_UART_Transmit(&huart2, (uint8_t*) str, str_len, HAL_MAX_DELAY);

		//str_len = sprintf(str, "Test2\r\n");
		//HAL_UART_Transmit(&huart2, (uint8_t*) str, str_len, HAL_MAX_DELAY);

		switch(state)
		{
		case s_init:
			state = s_all_stop;
			ticks_left_in_state = 2000;
			new_event 			= ev_none;
			str_len = sprintf(str, "Moving from s_init to s_all_stop\r\n");
			HAL_UART_Transmit(&huart2, (uint8_t*) str, str_len, HAL_MAX_DELAY);
		case s_all_stop:
			if (new_event == ev_state_timeout) {
				state 				= s_cars_go;
				ticks_left_in_state = 0;
				new_event 			= ev_none;
				set_traffic_lights(s_cars_go);
			}
			break;
		case s_cars_go:
			if (new_event == ev_button_press) {
				state 				= s_button_pressed;
				ticks_left_in_state = 3000;
				new_event 			= ev_none;
				set_traffic_lights(s_button_pressed);
			}
			break;
		case s_button_pressed:
			if (new_event == ev_state_timeout) {
				state 				= s_cars_stop;
				ticks_left_in_state = 2500;
				new_event 			= ev_none;
				set_traffic_lights(s_cars_stop);
			}
			break;
		case s_cars_stop:
			if (new_event == ev_state_timeout) {
				state				= s_pedestrians_go;
				ticks_left_in_state = 5000;
				new_event			= ev_none;
				set_traffic_lights(s_pedestrians_go);
			}
			break;
		case s_pedestrians_go:
			if (new_event == ev_state_timeout) {
				state 				= s_cars_start;
				ticks_left_in_state = 2500;
				new_event 			= ev_none;
				set_traffic_lights(s_cars_start);
			}
			break;
		case s_cars_start:
			if (new_event == ev_state_timeout) {
				state 				= s_cars_go;
				ticks_left_in_state = 0;
				new_event 			= ev_none;
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
