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
#include "NRF24L01.h"
#include "stdio.h"
#include "BMP180.h"
#include "AHT10.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SIZE_RX_BUF 5

nrf24 nrfRx;
uint8_t rx_data[SIZE_RX_BUF];
uint8_t rxAddr[] = { 0xEA, 0xDD, 0xCC, 0xBB, 0xAA };
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
	MX_SPI1_Init();
	MX_TIM2_Init();
	MX_I2C1_Init();
	MX_UART4_Init();
	/* USER CODE BEGIN 2 */
	nrfRx.CE_port = NRF_CE_GPIO_Port;
	nrfRx.CE_pin = NRF_CE_Pin;
	nrfRx.CSN_port = NRF_CSN_GPIO_Port;
	nrfRx.CSN_pin = NRF_CSN_Pin;
	nrfRx.IRQ_port = NRF_IRQ_GPIO_Port;
	nrfRx.IRQ_pin = NRF_IRQ_Pin;
	nrfRx.hSPIx = &hspi1;
	NRF24_Init(&nrfRx);
	NRF24_Set_DataRate(&nrfRx, _250KBS);
	NRF24_Set_PALevel(&nrfRx, HIGH);
	NRF24_Set_RxPipe(&nrfRx, rxAddr, 0, SIZE_RX_BUF);
	NRF24_Set_Mode(&nrfRx, RX_MODE);

	rx_data[0] = 0;
	rx_data[1] = 0;
	rx_data[2] = 0;
	rx_data[3] = 0;

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	float Temperature, Pressure, Altitude;
	char data[100];
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

//		Temperature = BMP180_GetTemp();
//		Pressure = BMP180_GetPress(1);
//		Altitude = BMP180_GetAlt(0);
//		sprintf(data, "Temp: %.2f\nPres: %.2f\nAlt: %.2f\n", Temperature, Pressure, Altitude);
//		HAL_UART_Transmit(&huart4,  (float*)data, sizeof(data), 100);
//		HAL_GPIO_WritePin(IN_Relay_GPIO_Port, IN_Relay_Pin, 1);
//		HAL_Delay(1000);
		if (NRF24_Available(&nrfRx, 0) == 1) {
			NRF24_Receive(&nrfRx, rx_data, SIZE_RX_BUF);
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

			if (rx_data[0] == 1) {
//				đi thẳng
				HAL_GPIO_WritePin(IN_1_GPIO_Port, IN_1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(IN_2_GPIO_Port, IN_2_Pin, GPIO_PIN_RESET);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1000);
				HAL_Delay(10);
				HAL_GPIO_WritePin(IN_3_GPIO_Port, IN_3_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(IN_4_GPIO_Port, IN_4_Pin, GPIO_PIN_RESET);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1000);
				HAL_Delay(10);
				rx_data[0] = 0;
				rx_data[1] = 0;
				rx_data[2] = 0;
				rx_data[3] = 0;
			} else if (rx_data[1] == 1) {
//				đi lùi
				HAL_GPIO_WritePin(IN_1_GPIO_Port, IN_1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(IN_2_GPIO_Port, IN_2_Pin, GPIO_PIN_SET);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1000);
				HAL_Delay(10);
				HAL_GPIO_WritePin(IN_3_GPIO_Port, IN_3_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(IN_4_GPIO_Port, IN_4_Pin, GPIO_PIN_SET);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1000);
				HAL_Delay(10);
				rx_data[0] = 0;
				rx_data[1] = 0;
				rx_data[2] = 0;
				rx_data[3] = 0;

			} else if (rx_data[2] == 1) {
				//				đi phải
				HAL_GPIO_WritePin(IN_1_GPIO_Port, IN_1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(IN_2_GPIO_Port, IN_2_Pin, GPIO_PIN_SET);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1000);
				HAL_Delay(10);
				HAL_GPIO_WritePin(IN_3_GPIO_Port, IN_3_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(IN_4_GPIO_Port, IN_4_Pin, GPIO_PIN_RESET);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1000);
				HAL_Delay(10);
				rx_data[0] = 0;
				rx_data[1] = 0;
				rx_data[2] = 0;
				rx_data[3] = 0;

			} else if (rx_data[3] == 1) {
				//				đi trái
				HAL_GPIO_WritePin(IN_1_GPIO_Port, IN_1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(IN_2_GPIO_Port, IN_2_Pin, GPIO_PIN_RESET);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1000);
				HAL_Delay(10);
				HAL_GPIO_WritePin(IN_3_GPIO_Port, IN_3_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(IN_4_GPIO_Port, IN_4_Pin, GPIO_PIN_SET);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1000);
				HAL_Delay(10);
				rx_data[0] = 0;
				rx_data[1] = 0;
				rx_data[2] = 0;
				rx_data[3] = 0;

			} else {
				HAL_GPIO_WritePin(IN_1_GPIO_Port, IN_1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(IN_2_GPIO_Port, IN_2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(IN_3_GPIO_Port, IN_3_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(IN_4_GPIO_Port, IN_4_Pin, GPIO_PIN_RESET);
				HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
				rx_data[0] = 0;
				rx_data[1] = 0;
				rx_data[2] = 0;
				rx_data[3] = 0;
			}
		}
		rx_data[0] = 0;
		rx_data[1] = 0;
		rx_data[2] = 0;
		rx_data[3] = 0;
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
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

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
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

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
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

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
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 72;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 1000;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief UART4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART4_Init(void) {

	/* USER CODE BEGIN UART4_Init 0 */

	/* USER CODE END UART4_Init 0 */

	/* USER CODE BEGIN UART4_Init 1 */

	/* USER CODE END UART4_Init 1 */
	huart4.Instance = UART4;
	huart4.Init.BaudRate = 115200;
	huart4.Init.WordLength = UART_WORDLENGTH_8B;
	huart4.Init.StopBits = UART_STOPBITS_1;
	huart4.Init.Parity = UART_PARITY_NONE;
	huart4.Init.Mode = UART_MODE_TX_RX;
	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart4.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart4) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN UART4_Init 2 */

	/* USER CODE END UART4_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, NRF_CE_Pin | NRF_CSN_Pin | LED_Pin | IN_Relay_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, IN_1_Pin | IN_2_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, IN_3_Pin | IN_4_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin : NRF_IRQ_Pin */
	GPIO_InitStruct.Pin = NRF_IRQ_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(NRF_IRQ_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : NRF_CE_Pin NRF_CSN_Pin LED_Pin IN_Relay_Pin */
	GPIO_InitStruct.Pin = NRF_CE_Pin | NRF_CSN_Pin | LED_Pin | IN_Relay_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : IN_1_Pin IN_2_Pin */
	GPIO_InitStruct.Pin = IN_1_Pin | IN_2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : IN_3_Pin IN_4_Pin */
	GPIO_InitStruct.Pin = IN_3_Pin | IN_4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : LED_1_Pin */
	GPIO_InitStruct.Pin = LED_1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_1_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
