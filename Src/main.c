
#include "main.h"
#include <stdio.h>
#include <stdlib.h>

#define front 20

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM10_Init(void);

void delay_micros(uint16_t micros) {
	__HAL_TIM_SET_COUNTER(&htim3, 0);  // Set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim3) < micros)
		;  // Wait for the counter to reach the us input in the parameter
}
long duration = 0;
int distance = 0;
int time_left = 2000;
int time_right = 1000;
int time_center = 0;
int right_distance = 0;
int left_distance = 0;

int main(void) {
	HAL_Init();

	SystemClock_Config();

	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_TIM10_Init();

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_Base_Start_IT(&htim4);

	HAL_GPIO_WritePin(right_IR_power_GPIO_Port, right_IR_power_Pin, 1);
	HAL_GPIO_WritePin(left_IR_power_GPIO_Port, left_IR_power_Pin, 1);
	HAL_GPIO_WritePin(right_cliff_power_GPIO_Port, right_cliff_power_Pin, 1);
	HAL_GPIO_WritePin(left_cliff_power_GPIO_Port, left_cliff_power_Pin, 1);

	while (1) {
		// If no obstacles near by and ground is detected, go straight
		if (distance >= front
				&& HAL_GPIO_ReadPin(right_IR_GPIO_Port, right_IR_Pin) == 1
				&& HAL_GPIO_ReadPin(left_IR_GPIO_Port, left_IR_Pin) == 1
				&& HAL_GPIO_ReadPin(right_cliff_GPIO_Port, right_cliff_Pin) == 0
				&& HAL_GPIO_ReadPin(left_cliff_GPIO_Port, left_cliff_Pin)
						== 0) {
			// Make sure wheel doesn't brake
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
			// Edward goes forward
			__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 110);	// neck servo faces forward
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1000);	// right wheel forward
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 1000);	// left wheel forward
		}
		// If cliff detected, back up, and then turn around ~180 deg
		else if (HAL_GPIO_ReadPin(right_cliff_GPIO_Port, right_cliff_Pin) == 1
				|| HAL_GPIO_ReadPin(left_cliff_GPIO_Port, left_cliff_Pin)
						== 1) {
			// Make sure wheel doesn't brake
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
			// Edward goes backward for 1000 ms
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1000);	// right wheel backward
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 1000);	// left wheel backward
			HAL_Delay(1000);
			// Edward turns around (to the right) for 1500 ms
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1000);	// right wheel forward
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 1000);	// left wheel backward
			HAL_Delay(1500);
		}
		// If left wall detected, turn right until said wall is not detected
		else if (HAL_GPIO_ReadPin(left_IR_GPIO_Port, left_IR_Pin) == 0) {
			// Make sure wheel doesn't brake
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
			// Edward turns right
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1000);	// right wheel backward
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 1000);	// left wheel forward
			/* If time between left and right turns is too small (within 500 ms),
			 * there is probably a corner. Turn ~180 deg */
			time_left = HAL_GetTick();
			if (abs(time_left - time_right) < 500) {
				// Edward turns around (to the right) for 1500 ms
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1000);	// right wheel backward
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 1000);	// left wheel forward
				HAL_Delay(1500);
			}
		}
		// If right wall detected, turn left until said wall is not detected
		else if (HAL_GPIO_ReadPin(right_IR_GPIO_Port, right_IR_Pin) == 0) {
			// Make sure wheel doesn't brake
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
			// Edward turns right
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1000);	// right wheel forward
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 1000);	// left wheel backward
			/* If time between left and right turns is too small,
			 * there is probably a corner. turn ~180 deg */
			time_right = HAL_GetTick();
			if (abs(time_right - time_left) < 500) {
				// Edward turns around (to the left) for 1500 ms
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1000);	// right wheel forward
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 1000);	// left wheel backward
				HAL_Delay(1500);
			}
		}
		/* If wall in front detected, look right, then look left,
		 * determine which way has the furthest obstacle, then go that way */
		else if (distance < front) {
			/* If time between left and right turns is too small,
			 * there is probably a corner. turn ~180 deg */
			time_center = HAL_GetTick();
			if (abs(time_center - time_left) < 500) {
				// Make sure wheel doesn't brake
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
				// Edward turns around (to the left) for 1500 ms
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1000);	// right wheel forward
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 1000);	// left wheel backward
				HAL_Delay(1500);
			}
			if (abs(time_center - time_right) < 500) {
				// Make sure wheel doesn't brake
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
				// Edward turns around (to the right) for 1500 ms
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1000);	// right wheel backward
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 1000);	// left wheel forward
				HAL_Delay(1500);
			}
			if (abs(time_center - time_left) >= 500
					&& abs(time_center - time_right) >= 500) {
				// Stop wheels
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
				// Neck looks right
				__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 40);
				HAL_Delay(1000);
				right_distance = distance;// Capture distance from right obstacles
				// Neck looks left
				__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 192);
				HAL_Delay(1000);
				left_distance = distance;// Capture distance from left obstacles
				// Neck looks forward
				__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 110);
				/* If there's more space to the right, Edward turns and goes to the right.
				 * If there's more space to the left, Edward turns and goes to the left. */
				if (right_distance >= left_distance) {
					// Edward turns to the right for 850 ms
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1000);	// right wheel backward
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 1000);	// left wheel forward
					HAL_Delay(850);
				} else {
					// Edward turns to the left for 850 ms
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1000);	// right wheel forward
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 1000);	// left wheel backward
					HAL_Delay(850);
				}
			}
		}
	}
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 80;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {
	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 100 - 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 1600 - 1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 25;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.Pulse = 0;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}

	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {
	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 80 - 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65536 - 1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {
	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 8000 - 1;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 10000 - 1;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief TIM10 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM10_Init(void) {
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	htim10.Instance = TIM10;
	htim10.Init.Prescaler = 1000 - 1;
	htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim10.Init.Period = 1600 - 1;
	htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim10) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim10) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	HAL_TIM_MspPostInit(&htim10);
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC,
			right_IR_power_Pin | my_trig_Pin | left_cliff_power_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(my_led_GPIO_Port, my_led_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(left_IR_power_GPIO_Port, left_IR_power_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(right_cliff_power_GPIO_Port, right_cliff_power_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin : my_button_Pin */
	GPIO_InitStruct.Pin = my_button_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(my_button_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : right_IR_Pin left_cliff_Pin right_cliff_Pin */
	GPIO_InitStruct.Pin = right_IR_Pin | left_cliff_Pin | right_cliff_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : right_IR_power_Pin my_trig_Pin left_cliff_power_Pin */
	GPIO_InitStruct.Pin = right_IR_power_Pin | my_trig_Pin
			| left_cliff_power_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : my_led_Pin */
	GPIO_InitStruct.Pin = my_led_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(my_led_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : left_IR_Pin */
	GPIO_InitStruct.Pin = left_IR_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(left_IR_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : left_IR_power_Pin */
	GPIO_InitStruct.Pin = left_IR_power_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(left_IR_power_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : my_echo_Pin */
	GPIO_InitStruct.Pin = my_echo_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(my_echo_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : right_cliff_power_Pin */
	GPIO_InitStruct.Pin = right_cliff_power_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(right_cliff_power_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim4) {
		HAL_GPIO_WritePin(my_trig_GPIO_Port, my_trig_Pin, 0);
		delay_micros(2);
		HAL_GPIO_WritePin(my_trig_GPIO_Port, my_trig_Pin, 1);
		delay_micros(10);
		HAL_GPIO_WritePin(my_trig_GPIO_Port, my_trig_Pin, 0);
		while (HAL_GPIO_ReadPin(my_echo_GPIO_Port, my_echo_Pin) == 0)
			;
		duration = 0;
		while (HAL_GPIO_ReadPin(my_echo_GPIO_Port, my_echo_Pin) == 1) {
			duration += 10;
			delay_micros(10);
		}
		distance = duration * 0.034 / 2;
		HAL_GPIO_TogglePin(my_led_GPIO_Port, my_led_Pin);
	}
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
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

}
#endif /* USE_FULL_ASSERT */
