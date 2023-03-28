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
#include "NRF24L01.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define front 20	/* Lowest distance between ultrasonic sensor and obstacle in
						front of it before Edward stops */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM10_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// NRF address for pairing
uint8_t rx_addr[] = { 0xEE, 0xDD, 0xCC, 0xBB, 0xAA };

// Variables for storing the received NRF data
uint8_t rx_data[32];
char MSG[100];

// Variables needed for ultrasonic sensor
long duration = 0;
int distance = 0;

// Variables needed for Edward to decide which direction to turn
int time_left = 2000;
int time_right = 1000;
int time_center = 0;
int right_distance = 0;
int left_distance = 0;

// Varibles for switching between autonomous and manual
int prev_button_val = 0;
int mode = 1;

// Variables needed for manual mode
int speed = 0;
int y_direction = 85;
int x_direction = 88;
int head_position = 110;
int prev_head_position = 0;

// Delays processing for microseconds
void delay_micros(uint16_t micros) {
	__HAL_TIM_SET_COUNTER(&htim2, 0);  // Set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim2) < micros)
		;  // Wait for the counter to reach the us input in the parameter
}

// Checks NRF data line for values
void check_NRF_rx() {
	if (is_data_available(1) == 1) {
		nrf24_receive(rx_data);
		HAL_UART_Transmit(&huart2, rx_data, strlen((char*) rx_data), 250);
		if (rx_data[16] == 'm' && rx_data[17] == '1' && prev_button_val == 0) {
			prev_button_val = 1;
			if (mode == 1) {
				mode = 0;
			} else {
				mode = 1;
			}
		} else if (rx_data[16] == 'm' && rx_data[17] == '0'
				&& prev_button_val == 1) {
			prev_button_val = 0;
		}
	}
}

// Maps one range of values to another
int map(uint8_t val, int in_min, int in_max, int out_min, int out_max) {
	return (int) (((val - in_min) * (out_max - out_min) / (in_max - in_min))
			+ out_min);
}

// Converts received data into proper integers
int convert(int val, int location) {
	switch (rx_data[(location + 1)]) {
	case '1':
		val = 100;
		break;
	case '2':
		val = 200;
		break;
	default:
		val = 0;
	}

	switch (rx_data[(location + 2)]) {
	case '1':
		val += 10;
		break;
	case '2':
		val += 20;
		break;
	case '3':
		val += 30;
		break;
	case '4':
		val += 40;
		break;
	case '5':
		val += 50;
		break;
	case '6':
		val += 60;
		break;
	case '7':
		val += 70;
		break;
	case '8':
		val += 80;
		break;
	case '9':
		val += 90;
		break;
	default:
		val += 0;
	}

	switch (rx_data[(location + 3)]) {
	case '1':
		val += 1;
		break;
	case '2':
		val += 2;
		break;
	case '3':
		val += 3;
		break;
	case '4':
		val += 4;
		break;
	case '5':
		val += 5;
		break;
	case '6':
		val += 6;
		break;
	case '7':
		val += 7;
		break;
	case '8':
		val += 8;
		break;
	case '9':
		val += 9;
		break;
	default:
		val += 0;
	}

	return val;
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_SPI2_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_TIM10_Init();
	/* USER CODE BEGIN 2 */

	// Start all needed timers and channels
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_Base_Start_IT(&htim4);

	// Power up needed sensors
	HAL_GPIO_WritePin(right_IR_power_GPIO_Port, right_IR_power_Pin, 1);
	HAL_GPIO_WritePin(left_IR_power_GPIO_Port, left_IR_power_Pin, 1);
	HAL_GPIO_WritePin(right_cliff_power_GPIO_Port, right_cliff_power_Pin, 1);
	HAL_GPIO_WritePin(left_cliff_power_GPIO_Port, left_cliff_power_Pin, 1);
	HAL_GPIO_WritePin(nrf_power_GPIO_Port, nrf_power_Pin, 1);
	HAL_Delay(1000);
	nrf24_init();
	nrf24_rx_mode(rx_addr, 30);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		check_NRF_rx();
		if (mode == 1) {	// Controlled mode
			// Data is parsed, converted, and used for their corresponding function
			if (rx_data[0] == 'h') {
				head_position = convert(head_position, 0);
				if (head_position > (prev_head_position + 1)
						|| head_position < (prev_head_position - 1))
					__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1,
							head_position);
				prev_head_position = head_position;
			}
			if (rx_data[4] == 's') {
				speed = convert(speed, 4);
				speed = map(speed, 0, 100, 500, 2000);
			}
			if (rx_data[8] == 'y') {
				y_direction = convert(y_direction, 8);
			}
			if (rx_data[12] == 'x') {
				x_direction = convert(x_direction, 12);
			}

			// If joystick in the center, Edward doesn't move
			if (y_direction > 80 && y_direction < 90 && x_direction > 83
					&& x_direction < 93) {
				HAL_GPIO_WritePin(green_led_GPIO_Port, green_led_Pin, RESET);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
				// If joystick is pushed forward, Edward moves forward
			} else if (y_direction >= 90 && x_direction > 83
					&& x_direction < 93) {
				HAL_GPIO_WritePin(green_led_GPIO_Port, green_led_Pin, SET);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);// right wheel forward
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, speed);// left wheel forward
				// If joystick is pushed backward, Edward moves backward
			} else if (y_direction <= 80 && x_direction > 83
					&& x_direction < 93) {
				HAL_GPIO_WritePin(green_led_GPIO_Port, green_led_Pin, SET);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed);// right wheel backward
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, speed);// left wheel backward
				// If joystick is pushed left, Edward moves left
			} else if (y_direction > 80 && y_direction < 90
					&& x_direction >= 93) {
				HAL_GPIO_WritePin(green_led_GPIO_Port, green_led_Pin, SET);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);// right wheel forward
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, speed);// left wheel backward
				// If joystick is pushed right, Edward moves right
			} else if (y_direction > 80 && y_direction < 90
					&& x_direction <= 83) {
				HAL_GPIO_WritePin(green_led_GPIO_Port, green_led_Pin, SET);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed);// right wheel backward
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, speed);// left wheel forward
				// If joystick is pushed forward and to the left, Edward turns a bit to the left
			} else if (y_direction >= 100 && x_direction >= 91) {
				HAL_GPIO_WritePin(green_led_GPIO_Port, green_led_Pin, SET);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);// right wheel forward
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (speed / 2));// left wheel forward slower
				// If joystick is pushed backward and to the left, Edward backs up and turns a bit to the left
			} else if (y_direction <= 70 && x_direction >= 103) {
				HAL_GPIO_WritePin(green_led_GPIO_Port, green_led_Pin, SET);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed);// right wheel backward
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (speed / 2));// left wheel backward slower
				// If joystick is pushed forward and to the right, Edward turns a bit to the right
			} else if (y_direction >= 100 && x_direction <= 83) {
				HAL_GPIO_WritePin(green_led_GPIO_Port, green_led_Pin, SET);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (speed / 2));// right wheel forward slower
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, speed);// left wheel forward
				// If joystick is pushed backward and to the right, Edward backs up and turns a bit to the right
			} else if (y_direction <= 70 && x_direction <= 73) {// backward-right
				HAL_GPIO_WritePin(green_led_GPIO_Port, green_led_Pin, SET);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (speed / 2));// right wheel backward
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, speed);// left wheel backward
			}

		} else if (mode == 0) {	// autonomous mode
			check_NRF_rx();
			if (distance >= front
					&& HAL_GPIO_ReadPin(right_IR_GPIO_Port, right_IR_Pin) == 1
					&& HAL_GPIO_ReadPin(left_IR_GPIO_Port, left_IR_Pin) == 1
					&& HAL_GPIO_ReadPin(right_cliff_GPIO_Port, right_cliff_Pin)
							== 0
					&& HAL_GPIO_ReadPin(left_cliff_GPIO_Port, left_cliff_Pin)
							== 0) {
				// Make sure wheel doesn't brake
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
				// Edward goes forward
				__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 110);	// neck servo faces forward
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000);	// right wheel forward
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1000);	// left wheel forward
			}
			// If cliff detected, back up, and then turn around ~180 deg
			else if (HAL_GPIO_ReadPin(right_cliff_GPIO_Port, right_cliff_Pin)
					== 1
					|| HAL_GPIO_ReadPin(left_cliff_GPIO_Port, left_cliff_Pin)
							== 1) {
				// Make sure wheel doesn't brake
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
				// Edward goes backward for 1000 ms
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1000);	// right wheel backward
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1000);	// left wheel backward
				check_NRF_rx();
				HAL_Delay(1000);
				// Edward turns around (to the right) for 1500 ms
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000);	// right wheel forward
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1000);	// left wheel backward
				check_NRF_rx();
				HAL_Delay(1500);
			}
			// If left wall detected, turn right until said wall is not detected
			else if (HAL_GPIO_ReadPin(left_IR_GPIO_Port, left_IR_Pin) == 0) {
				// Make sure wheel doesn't brake
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
				// Edward turns right
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1000);	// right wheel backward
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1000);	// left wheel forward
				/* If time between left and right turns is too small (within 500 ms),
				 * there is probably a corner. Turn ~180 deg */
				time_left = HAL_GetTick();
				if (abs(time_left - time_right) < 500) {
					// Edward turns around (to the right) for 1500 ms
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1000);	// right wheel backward
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1000);	// left wheel forward
					check_NRF_rx();
					HAL_Delay(1500);
				}
			}
			// If right wall detected, turn left until said wall is not detected
			else if (HAL_GPIO_ReadPin(right_IR_GPIO_Port, right_IR_Pin) == 0) {
				// Make sure wheel doesn't brake
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
				// Edward turns right
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000);	// right wheel forward
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1000);	// left wheel backward
				/* If time between left and right turns is too small,
				 * there is probably a corner. turn ~180 deg */
				time_right = HAL_GetTick();
				if (abs(time_right - time_left) < 500) {
					// Edward turns around (to the left) for 1500 ms
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000);	// right wheel forward
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1000);	// left wheel backward
					check_NRF_rx();
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
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
					// Edward turns around (to the left) for 1500 ms
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000);	// right wheel forward
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1000);	// left wheel backward
					check_NRF_rx();
					HAL_Delay(1500);
				}
				if (abs(time_center - time_right) < 500) {
					// Make sure wheel doesn't brake
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
					// Edward turns around (to the right) for 1500 ms
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1000);	// right wheel backward
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1000);	// left wheel forward
					check_NRF_rx();
					HAL_Delay(1500);
				}
				if (abs(time_center - time_left) >= 500
						&& abs(time_center - time_right) >= 500) {
					// Stop wheels
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
					__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
					// Neck looks right
					__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 40);
					check_NRF_rx();
					HAL_Delay(1000);
					right_distance = distance;// Capture distance from right obstacles
					// Neck looks left
					__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 192);
					check_NRF_rx();
					HAL_Delay(1000);
					left_distance = distance; // Capture distance from left obstacles
					// Neck looks forward
					__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 110);
					/* If there's more space to the right, Edward turns and goes to the right.
					 * If there's more space to the left, Edward turns and goes to the left. */
					if (right_distance >= left_distance) {
						// Edward turns to the right for 850 ms
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1000);	// right wheel backward
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1000);	// left wheel forward
						check_NRF_rx();
						HAL_Delay(850);
					} else {
						// Edward turns to the left for 850 ms
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000);	// right wheel forward
						__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1000);	// left wheel backward
						check_NRF_rx();
						HAL_Delay(850);
					}
				}
			}
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
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void) {

	/* USER CODE BEGIN SPI2_Init 0 */

	/* USER CODE END SPI2_Init 0 */

	/* USER CODE BEGIN SPI2_Init 1 */

	/* USER CODE END SPI2_Init 1 */
	/* SPI2 parameter configuration*/
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI2_Init 2 */

	/* USER CODE END SPI2_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 80 - 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 65536 - 1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 100 - 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 1600 - 1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 25;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.Pulse = 0;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
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
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief TIM10 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM10_Init(void) {

	/* USER CODE BEGIN TIM10_Init 0 */

	/* USER CODE END TIM10_Init 0 */

	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM10_Init 1 */

	/* USER CODE END TIM10_Init 1 */
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
	/* USER CODE BEGIN TIM10_Init 2 */

	/* USER CODE END TIM10_Init 2 */
	HAL_TIM_MspPostInit(&htim10);

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

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
	if (HAL_UART_Init(&huart2) != HAL_OK) {
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
	right_IR_power_Pin | trig_Pin | left_cliff_power_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(green_led_GPIO_Port, green_led_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
	left_IR_power_Pin | CE_Pin | CSN_Pin | nrf_power_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(right_cliff_power_GPIO_Port, right_cliff_power_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin : blue_button_Pin */
	GPIO_InitStruct.Pin = blue_button_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(blue_button_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : right_IR_Pin left_IR_Pin left_cliff_Pin right_cliff_Pin */
	GPIO_InitStruct.Pin = right_IR_Pin | left_IR_Pin | left_cliff_Pin
			| right_cliff_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : right_IR_power_Pin trig_Pin left_cliff_power_Pin */
	GPIO_InitStruct.Pin = right_IR_power_Pin | trig_Pin | left_cliff_power_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : green_led_Pin */
	GPIO_InitStruct.Pin = green_led_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(green_led_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : left_IR_power_Pin CE_Pin CSN_Pin nrf_power_Pin */
	GPIO_InitStruct.Pin = left_IR_power_Pin | CE_Pin | CSN_Pin | nrf_power_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : echo_Pin */
	GPIO_InitStruct.Pin = echo_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(echo_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : right_cliff_power_Pin */
	GPIO_InitStruct.Pin = right_cliff_power_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(right_cliff_power_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

// Ultrasonic sensor takes measurement every second
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim4) {
		HAL_GPIO_WritePin(trig_GPIO_Port, trig_Pin, 0);
		delay_micros(2);
		HAL_GPIO_WritePin(trig_GPIO_Port, trig_Pin, 1);
		delay_micros(10);
		HAL_GPIO_WritePin(trig_GPIO_Port, trig_Pin, 0);
		while (HAL_GPIO_ReadPin(echo_GPIO_Port, echo_Pin) == 0)
			;
		duration = 0;
		while (HAL_GPIO_ReadPin(echo_GPIO_Port, echo_Pin) == 1) {
			duration += 10;
			delay_micros(10);
		}
		distance = duration * 0.034 / 2;
		HAL_GPIO_TogglePin(green_led_GPIO_Port, green_led_Pin);
	}
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
