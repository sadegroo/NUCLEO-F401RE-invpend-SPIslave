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
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// Global flags
volatile uint8_t uart_xmit_flag = 0;
volatile uint8_t uart_err_flag = 0;
volatile uint8_t spi_xmit_flag = 0;
volatile uint8_t spi_recv_flag = 0;
volatile uint8_t spi_txrx_flag = 0;
volatile uint8_t spi_err_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
int encoder_position_read(Quadrature_Encoder_TypeDef *encoder, TIM_HandleTypeDef *htim3);
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
  char uart_buf[UART_BUFFER_SIZE];
  int  uart_buf_len;
  int encoder_range_error;

  char spi_rx_buf[SPI_BUFFER_SIZE];
  char spi_tx_buf[SPI_BUFFER_SIZE];
  char spi_rx_buf_copy[SPI_BUFFER_SIZE];
  float recv_number=0.0;
  float send_number;
  uint8_t state_spi = 0;
  uint8_t state_uart = 0;
  uint32_t suc_cnt = 0;
  uint32_t err_cnt = 0;
  uint32_t buf_iter = 0;

  for(int i=0;i<SPI_BUFFER_SIZE;i++)
  {
	  spi_rx_buf[i]=0;
  }

  //Declare and initialize encoder
  Quadrature_Encoder_TypeDef encoder_inst = {0,0,0,0,0,0,0,0,0,0,0,0,FALSE,FALSE,FALSE,2400};
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
  MX_SPI3_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  // Say something
  uart_buf_len = sprintf(uart_buf, "SPI Interrupt Test\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);

  // Initialize encoder
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // UART state machine
		switch (state_uart)
		{
		  case 0:
			  // Idle
			  if ((suc_cnt % UART_DECIMATION) == 0 && suc_cnt != 0 )
			  {
				  state_uart = 1;
				  buf_iter = 0;
			  }
			  break;

		  case 1:
			// Print out bytes

			switch(HAL_UART_GetState(&huart2))
			{
			case HAL_UART_STATE_READY:
			  uart_buf_len = sprintf(uart_buf, "0x%02x ",(unsigned int)spi_rx_buf_copy[buf_iter]);
			  HAL_UART_Transmit_IT(&huart2, (uint8_t *)uart_buf, uart_buf_len);
			  state_uart = 2;
			default:
			  break;
			}
			break;

		  case 2:
			  // wait for transmit flag
			  if (uart_xmit_flag)
			  {
				  uart_xmit_flag = 0;
				  buf_iter++;
				  if (buf_iter == SPI_BUFFER_SIZE)
				  {
					  state_uart = 3;
					  break;
				  } else
				  {
					  state_uart = 1;
					  break;
				  }
			  }
			  if (uart_err_flag && HAL_UART_GetState(&huart2) == HAL_UART_STATE_READY)
			  {
			  // Clear flag and try again
				uart_err_flag = 0;
				state_uart = 1;
			  }
			  break;

		  case 3:

			// Print newline
				switch(HAL_UART_GetState(&huart2))
				{
				case HAL_UART_STATE_READY:
					uart_buf_len = sprintf(uart_buf, "success: %d, errors: %d, rxnumber = %.3f, txnumber = %.3f\r\n", (unsigned int)suc_cnt, (unsigned int)err_cnt, recv_number, send_number);
					HAL_UART_Transmit_IT(&huart2, (uint8_t *)uart_buf, uart_buf_len);

				default:
				  break;
				}

			state_uart = 4;
			break;

		  case 4:
			  // wait for transmit flag
			  if (uart_xmit_flag)
			  {
				  uart_xmit_flag =0;
				  state_uart = 0;
				  break;
			  }
			  if (uart_err_flag && HAL_UART_GetState(&huart2) == HAL_UART_STATE_READY)
			  {
			  // Clear flag and try again
				uart_err_flag = 0;
				state_uart = 1;
				break;
			  }
		  default:
			break;

		} //close switch

		// Finite state machine to allow for non-blocking SPI transmit/receive
		switch(state_spi)
		{
		  case 0:
		  // transmit and receive
			switch(HAL_SPI_GetState(&hspi3))
			{
			case HAL_SPI_STATE_READY:
				// Read encoder
				encoder_range_error = encoder_position_read(&encoder_inst, &htim3);
				send_number = encoder_inst.position;
				memcpy(spi_tx_buf, &send_number , sizeof(send_number));
				HAL_SPI_TransmitReceive_IT(&hspi3, (uint8_t *)spi_tx_buf, (uint8_t *)spi_rx_buf, SPI_BUFFER_SIZE);
				// Go to next state: waiting for interrupt flag
				state_spi = 1;
				break;
			default:
				break;
			}

		  // Wait for transmit/receive flag
		  case 1:
			  if (spi_txrx_flag)
			  {
			  // Clear flag and go to next state
					spi_txrx_flag = 0;
					state_spi = 0;
					suc_cnt++;
					// copy buffer
					memcpy(spi_rx_buf_copy, spi_rx_buf, sizeof(spi_rx_buf));
					memcpy(&recv_number, spi_rx_buf_copy, sizeof(recv_number));

				}
			  // catch error
			  if (spi_err_flag && HAL_SPI_GetState(&hspi3) == HAL_SPI_STATE_READY)
			  {
			  // Clear flag and try again
				spi_err_flag = 0;
				state_spi = 0;
				err_cnt++;
				// HAL_Delay(1);
			  }
			  break;
		  default:
			  break;
			}

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
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_SLAVE;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

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

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/*
 * Encoder position read (returns signed integer)
 *
 * Includes capability for tracking pendulum excursion for Swing Up control.
 * This is developed and provided by Markus Dauberschmidt.  Please see
 * https://github.com/OevreFlataeker/steval_edukit_swingup
 *
 */

int encoder_position_read(Quadrature_Encoder_TypeDef *encoder, TIM_HandleTypeDef *htim3) {

	int range_error = 0;
	encoder->cnt3 = __HAL_TIM_GET_COUNTER(htim3);

	if (encoder->cnt3 >= 32768) {
		encoder->position_steps = (int) (encoder->cnt3);
		encoder->position_steps = encoder->position_steps - 65536;
	} else {
		encoder->position_steps = (int) (encoder->cnt3);
	}

	if (encoder->position_steps <= -32768) {
		range_error = -1;
		encoder->position_steps = -32768;
	}
	if (encoder->position_steps >= 32767) {
		range_error = 1;
		encoder->position_steps = 32767;
	}
	// Subtract zeroing offset
	encoder->position_steps = encoder->position_steps - encoder->position_init;
	// calculate in position in radians
	encoder->position = (float) encoder->position_steps / (float) encoder->COUNTS_PER_TURN * 2.0f * (float) M_PI;

	/*
	 *  Detect if we passed the bottom, then re-arm peak flag
	 *  oppositeSigns returns true when we pass the bottom position
	 */


	if (HAS_OPPOSITE_SIGNS(encoder->position_steps, encoder->previous_position))
	{
		encoder->peaked = FALSE;
		encoder->zero_crossed = TRUE;
	}

	if (!encoder->peaked) // We don't need to evaluate anymore if we hit a maximum when we're still in downward motion and didn't cross the minimum
	{
		// Add global maximum
		if (abs(encoder->position_steps) >= abs(encoder->global_max_position))
		{
			encoder->global_max_position = encoder->position;
		}
		// Check if new maximum
		if (abs(encoder->position_steps) >= abs(encoder->max_position))
		{
			encoder->max_position = encoder->position_steps;
		}
		else
		{
			// We are at the peak and disable further checks until we traversed the minimum position again
			encoder->peaked = TRUE;
			encoder->handled_peak = FALSE;
		}
	}

	encoder->previous_position = encoder->position_steps;


	return range_error;
}

// This is called when SPI transmit is done
void HAL_SPI_TxCpltCallback (SPI_HandleTypeDef * hspi)
{
  // Set CS pin to high and raise flag
  spi_xmit_flag = 1;
}

// This is called when SPI receive is done
void HAL_SPI_RxCpltCallback (SPI_HandleTypeDef * hspi)
{
  // Set CS pin to high and raise flag
  spi_recv_flag = 1;
}

void HAL_SPI_TxRxCpltCallback (SPI_HandleTypeDef * hspi)
{
  spi_txrx_flag = 1;
}

void HAL_SPI_ErrorCallback (SPI_HandleTypeDef * hspi)
{
  // Set CS pin to high and raise flag
  spi_err_flag = 1;
}

void HAL_UART_TxCpltCallback (UART_HandleTypeDef * huart)
{
  // Set CS pin to high and raise flag
  uart_xmit_flag = 1;
}

void HAL_UART_ErrorCallback (UART_HandleTypeDef * huart)
{
  // Set CS pin to high and raise flag
	uart_err_flag = 1;
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
