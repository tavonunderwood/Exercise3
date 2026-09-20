/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

//Redefines buttons for simplicity
// Positive x direction is forward
//Positive y direction is left
//positive z direction is up
//Left movement port and pin
#define LEFT_Pin A1_Pin
#define RIGHT_Port A2_GPIO_Port
//Right movement port and pin
#define RIGHT_Port A1_GPIO_Port
#define RIGHT_Pin A2_PIN
//Backward movement port and pin
#define BACK_Pin A3_Pin
#define BACK_Port A3_GPIO_Port
//Forward movement port and pin
#define FOR_Port A4_Pin
#define FOR_Pin A4_GPIO_Port
//Up down output port and pin
#define ZPUL_Port D5_GPIO_Port
#define ZPUL_Pin D5_Pin
//Up down direction port and pin
#define ZDIR_Port D4_GPIO_Port
#define ZDIR_Pin D4_Pin
//Forward back output
#define XPUL_Port D3_GPIO_Port
#define XPUL_Pin D3_Pin
// Forward back direction
#define XDIR_Port D2_GPIO_Port
#define XDIR_Pin D2_Pin
// Left right output
#define YPUL_Port D1_GPIO_Port
#define YPUL_Pin D1_Pin
//Left Right direction
#define YDIR_Port D0_GPIO_Port
#define YDIR_Pin D0_Pin
// Z axis proximity sensor
#define ZPROX_Port D8_GPIO_Port
#define ZPROX_Pin D8_Pin
// Y axis proximity sensor
#define YPROX_Port D9_GPIO_Port
#define YPROX_Pin D9_Pin
//x axis proximity sensor
#define XPROX_Port D10_GPIO_Port
#define XPROX_Pin D10_Pin



/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/*What period should we use to achieve 22 m/s?
 * T = ( L(lead in mm/rev)* Steps_Per_Rev(200 for out gantry system) )/ (Desired Speed: 22 mm/s) = steps/sec
 * L = 8mm, 200 steps per rev, V = 22 mm/s : 550 steps/ sec
 * Take reciprocal: 0.018 seconds/step = 1.8 ms
 * Use 1.8ms for high and low
 *
 */



#define stepInterval 1.8f


GPIO_PinState leftButtonState;
GPIO_PinState rightButtonState;
GPIO_PinState forwardButtonState;
GPIO_PinState backButtonState;
//Records time since recent step to produce signal
uint32_t lastXStep = 0;
uint32_t lastYStep = 0;
uint32_t lastZStep = 0;

//Alternates between on and off to produce signal
uint8_t xStepState = 0;
uint8_t yStepState = 0;
uint8_t zStepState = 0;

//State of 2mm proximity
uint8_t zProximityDetected = 0;
uint8_t yProximityDetected = 0;
uint8_t xProximityDetected = 0;

//Determines calibration state of each axis

uint8_t zCalibrated = 0;
uint8_t yCalibrated = 0;
uint8_t zCalibrated = 0;

// enums that state machine utilizes to calibrate

typedef enum
{
	Z_IDLE,
	Z_MOVING_DOWN,
	Z_WAITING,
	Z_MOVING_UP
} ZState;

typedef enum
{
	Y_IDLE,
	Y_MOVING_RIGHT,
	Y_WAITING,
	Y_MOVING_LEFT
} YState;

typedef enum
{
	X_IDLE,
	X_MOVING_BACK,
	X_WAITING,
	X_MOVING_FORWARD
} XState;

//Initialize states
ZState = IDLE;
YState = IDLE;
XState = IDLE;

//Global origin values

uint8_32 yOrigin;
uint8_32 yOrigin;
uint8_32 zOrigin;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void SystemPower_Config(void);
static void MX_GPIO_Init(void);
static void MX_ICACHE_Init(void);
static void MX_USART1_UART_Init(void);
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

  /* Configure the System Power */
  SystemPower_Config();

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ICACHE_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {
	  leftButtonState = HAL_GPIO_ReadPin(LEFT_Port, LEFT_Pin);
	  rightButtonState = HAL_GPIO_ReadPin(RIGHT_Port, RIGHT_Pin);
	  backButtonState = HAL_GPIO_ReadPin(BACK_Port, Back_Pin);
	  forwardButtonState = HAL_GPIO_ReadPin(FORWARD_Port, FORWARD_Pin);
	  //Loop until right button is pressed
	  while(rightButtonState == GPIO_PIN_RESET)
	  {
		  if (leftButtonState == GPIO_PIN_SET)
		  {
			  if (HAL_GetTick() - lastYStep >= stepInterval)
			  {
				  yStepState = !yStepState;
				  HAL_GPIO_WritePin(LEFT_Port, LEFT_Pin, yStepState? GPIO_PIN_SET : GPIO_PIN_RESET);
				  lastYStep = HAL_GetTick();
			  }
		  }
		  if (forwardButtonState == GPIO_PIN_SET)
		  {
			  HAL_GPIO_WritePin(YDIR_GPIO_Port, YDIR_Pin, GPIO_PIN_RESET);
			  if (HAL_GetTick() - lastYStep >= stepInterval)
			  {
				  yStepState = !yStepState;
				  HAL_GPIO_WritePin(YPUL_Port, YPUL_Pin, yStepState? GPIO_PIN_SET : GPIO_PIN_RESET);
				  lastYStep = HAL_GetTick();
			  }
		  }
	  }
	 //Calibrate the sensors
	  Z_Calibration();
	  Y_Calibration();
	  X_Calibration();


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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_0;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV4;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Power Configuration
  * @retval None
  */
static void SystemPower_Config(void)
{
  HAL_PWREx_EnableVddIO2();

  /*
   * Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral
   */
  HAL_PWREx_DisableUCPDDeadBattery();

  /*
   * Switch to SMPS regulator instead of LDO
   */
  if (HAL_PWREx_ConfigSupply(PWR_SMPS_SUPPLY) != HAL_OK)
  {
    Error_Handler();
  }
/* USER CODE BEGIN PWR */
/* USER CODE END PWR */
}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache in 1-way (direct mapped cache)
  */
  if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, D4_Pin|D2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, D5_Pin|D3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, LED_RED_Pin|D1_Pin|D0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_BUTTON_Pin */
  GPIO_InitStruct.Pin = USER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : A4_Pin A1_Pin */
  GPIO_InitStruct.Pin = A4_Pin|A1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : AS_Pin BS_Pin */
  GPIO_InitStruct.Pin = AS_Pin|BS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : A2_Pin */
  GPIO_InitStruct.Pin = A2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(A2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : A3_Pin */
  GPIO_InitStruct.Pin = A3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(A3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : D8_Pin */
  GPIO_InitStruct.Pin = D8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(D8_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : D4_Pin D2_Pin */
  GPIO_InitStruct.Pin = D4_Pin|D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : D5_Pin D3_Pin */
  GPIO_InitStruct.Pin = D5_Pin|D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : D10_Pin D9_Pin */
  GPIO_InitStruct.Pin = D10_Pin|D9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_RED_Pin D1_Pin D0_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin|D1_Pin|D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_BLUE_Pin */
  GPIO_InitStruct.Pin = LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_BLUE_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI12_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI12_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//Calibrates the Z-Origin
void Z_Calibration()
{
	  //Begin Z state machine
	  switch(ZState)
	  {
	  	  case Z_IDLE:
		  {
	  		  //Not necessary here
		  }
	  	  case Z_MOVING_DOWN:
		  {
			  //Changed by interrupt
			  if (zProximityDetected)
			  {
				  // Turns motor off
				  HAL_GPIO_WritePin(ZPUL_Port, ZPUL_Pin, GPIO_PIN_RESET);

				  //resets step state to zero for future use
				  zStepState = 0;


				  zState = Z_MOVING_UP;
			  }
			  // Moves down
			  else
			  {
				  //Sets direction to down (hopefully)
				  HAL_GPIO_WritePin(ZDIR_Port, ZDIR_Pin, GPIO_PIN_RESET);

				  //3.6 ms clock
				  if (HAL_GetTick() - zLastStepTime >= stepInterval)
				  {
					  //switches step state (on or off)
					  zStepState = !zStepState;

					  HAL_GPIO_WritePin(ZPUL_Port, Z_Pin, zStepState ? GPIO_PIN_SET : GPIO_PIN_RESET);
					  //Resets last step time
					  zLastStepTime = HAL_GetTick();
				  }
			  }
			  break;
		  }
	  	  case Z_WAITING:
		  {
	  		  //Not necessary here
		  }
	  	  case Z_MOVING_UP:
		  {
			  //Move upward for 0.5 seconds to reach 10 mm
			  //Set direction to up
			  HAL_GPIO_WritePin(ZDIR_Port, ZDIR_Pin, GPIO_PIN_SET);
			  if (HAL_GetTick() - zLastStepTime >= 500)
			  {
				  zCalibrated = 1;
			  }
		  }

	  }
}

void Y_Calibration()
{
	  //Begin Z state machine
	  switch(yState)
	  {
	  	  case Y_IDLE:
		  {
	  		  if (zCalibrated == 1)
	  		  {
	  			  yState = Y_MOVING_RIGHT;
	  		  }
		  }
	  	  case Y_MOVING_RIGHT:
		  {
			  //Changed by interrupt
			  if (yProximityDetected)
			  {
				  // Turns motor off
				  HAL_GPIO_WritePin(YPUL_Port, YPUL_Pin, GPIO_PIN_RESET);

				  //resets step state to zero for future use
				  yStepState = 0;


				  yState = Y_MOVE_LEFT;
			  }
			  // Moves down
			  else
			  {
				  //Sets direction to down (hopefully)
				  HAL_GPIO_WritePin(YDIR_Port, YDIR_Pin, GPIO_PIN_RESET);

				  //3.6 ms clock
				  if (HAL_GetTick() - yLastStepTime >= stepInterval)
				  {
					  //switches step state (on or off)
					  yStepState = !yStepState;

					  HAL_GPIO_WritePin(YPUL_Port, Y_Pin, yStepState ? GPIO_PIN_SET : GPIO_PIN_RESET);
					  //Resets last step time
					  yLastStepTime = HAL_GetTick();
				  }
			  }
			  break;
		  }
	  	  case Y_WAITING:
		  {
	  		  //Not necessary here
		  }
	  	  case Y_MOVING_LEFT:
		  {
			  //Move upward for 0.5 seconds to reach 10 mm
			  //Set direction to up
			  HAL_GPIO_WritePin(ZDIR_Port, ZDIR_Pin, GPIO_PIN_SET);
			  if (HAL_GetTick() - zLastStepTime >= 500)
			  {
				  zCalibrated = 1;
			  }
		  }
	  }
}

void X_Calibration()
{
	  //Begin Z state machine
	  switch(xState)
	  {
	  	  case X_IDLE:
		  {
	  		  if (yCalibrated == 1)
	  		  {
	  			  xState = X_MOVING_BACK;
	  		  }
		  }
	  	  case Y_MOVING_BACK:
		  {
			  //Changed by interrupt
			  if (xProximityDetected)
			  {
				  // Turns motor off
				  HAL_GPIO_WritePin(XPUL_Port, XPUL_Pin, GPIO_PIN_RESET);

				  //resets step state to zero for future use
				  xStepState = 0;

				  xState = X_MOVING_FORWARD;
			  }
			  // Moves down
			  else
			  {
				  //Sets direction to down (hopefully)
				  HAL_GPIO_WritePin(XDIR_Port, XDIR_Pin, GPIO_PIN_RESET);

				  //3.6 ms clock
				  if (HAL_GetTick() - xLastStepTime >= stepInterval)
				  {
					  //switches step state (on or off)
					  xStepState = !xStepState;

					  HAL_GPIO_WritePin(XPUL_Port, X_Pin, xStepState ? GPIO_PIN_SET : GPIO_PIN_RESET);
					  //Resets last step time
					  xLastStepTime = HAL_GetTick();
				  }
			  }
			  break;
		  }
	  	  case X_WAITING:
		  {
	  		  //Not necessary here
		  }
	  	  case X_MOVING_FORWARD:
		  {
			  //Move upward for 0.5 seconds to reach 10 mm
			  //Set direction to up
			  HAL_GPIO_WritePin(XDIR_Port, XDIR_Pin, GPIO_PIN_SET);
			  if (HAL_GetTick() - xLastStepTime >= 500)
			  {
				  xCalibrated = 1;
			  }
		  }
	  }
}

//Interrupt triggered externally
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
	UNUSED(GPIO_Pin);

	// Z-axis proximity pin triggered
	if (GPIO_Pin == ZPROX_Pin)
	{
		//State machine reads this
		zProxmityDetected = 1;
		HAL_GPIO_WritePin(ZPUL_Port, ZPUL_Pin, GPIO_PIN_RESET);

	}
	//Symmetrical
	else if (GPIO_Pin == YPROX_Pin)
	{
		yProximityDetected = 1;

		HAL_GPIO_WritePin(ZPUL_Port, ZPUL_Pin, GPIO_PIN_RESET);
	}
	else if (GPIO_Pin == XPROX_Pin)
	{
		xProximityDetected = 1;
		HAL_GPIO_WritePin(ZPUL_Port, ZPUL_Pin, GPIO_PIN_RESET);
	}

}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM17 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM17)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
#ifdef USE_FULL_ASSERT
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
