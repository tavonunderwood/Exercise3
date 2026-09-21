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
#define LEFT_Port      A1_GPIO_Port
#define LEFT_Pin       A1_Pin
//Right movement port and pin
#define RIGHT_Port     A2_GPIO_Port
#define RIGHT_Pin      A2_Pin
//Backward movement port and pin
#define BACK_Port      A3_GPIO_Port
#define BACK_Pin       A3_Pin
//Forward movement port and pin
#define FORWARD_Port   A4_GPIO_Port
#define FORWARD_Pin    A4_Pin
//Up down output port and pin
#define ZPUL_Port      D5_GPIO_Port
#define ZPUL_Pin       D5_Pin
#define ZDIR_Port      D4_GPIO_Port
#define ZDIR_Pin       D4_Pin
//Forward back output/direction
#define XPUL_Port      D3_GPIO_Port
#define XPUL_Pin       D3_Pin
#define XDIR_Port      D2_GPIO_Port
#define XDIR_Pin       D2_Pin
// Left right output
#define YPUL_Port      D1_GPIO_Port
#define YPUL_Pin       D1_Pin
#define YDIR_Port      D0_GPIO_Port
#define YDIR_Pin       D0_Pin
// Z axis proximity sensor
#define ZPROX_Port     D8_GPIO_Port
#define ZPROX_Pin      D8_Pin
// Y axis proximity sensor
#define YPROX_Port     D9_GPIO_Port
#define YPROX_Pin      D9_Pin
//x axis proximity sensor
#define XPROX_Port     D10_GPIO_Port
#define XPROX_Pin      D10_Pin



/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim3;

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
volatile uint8_t zProximityDetected = 0;
volatile uint8_t yProximityDetected = 0;
volatile uint8_t xProximityDetected = 0;

//Determines calibration state of each axis

uint8_t zCalibrated = 0;
uint8_t yCalibrated = 0;
uint8_t xCalibrated = 0;

// enums that state machine utilizes to calibrate

typedef enum
{
    CAL_Z_DOWN,
    CAL_Z_UP,
    CAL_Y_RIGHT,
    CAL_Y_LEFT,
    CAL_X_BACK,
    CAL_X_FORWARD,
    CAL_WAIT,
    CAL_DONE
} CalibrationState;

CalibrationState calState = CAL_Z_DOWN;
//Initialize states
ZState zState = Z_IDLE;
YState yState = Y_IDLE;
XState xState = X_IDLE;

//Global origin values

int32_t xPosition = 0;
int32_t yPosition = 0;
int32_t zPosition = 0;

//Defines step parameters
#define FULL_STEPS_PER_REV    200
#define MICROSTEP             2
#define PULSES_PER_REV        (FULL_STEPS_PER_REV * MICROSTEP)

#define LEAD_MM_PER_REV       8
#define PULSES_PER_MM         (PULSES_PER_REV / LEAD_MM_PER_REV)

#define Z_BACKOFF_MM          10
#define Y_BACKOFF_MM          100

#define Z_BACKOFF_PULSES      (Z_BACKOFF_MM * PULSES_PER_MM)
#define Y_BACKOFF_PULSES      (Y_BACKOFF_MM * PULSES_PER_MM)

//For hardware interrupt
//Can now turn this on and off to control motor
volatile uint8_t zMotorRunning = 0;
volatile uint8_t yMotorRunning = 0;
volatile uint8_t xMotorRunning = 0;

//For moving up 10mm
volatile uint32_t zPulseCount = 0;

//for moving left 10mm
volatile uint32_t yPulseCount = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void SystemPower_Config(void);
static void MX_GPIO_Init(void);
static void MX_ICACHE_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
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
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  //Startup Loop
  while (1)
  {
      leftButtonState = HAL_GPIO_ReadPin(LEFT_Port, LEFT_Pin);

      rightButtonState = HAL_GPIO_ReadPin(RIGHT_Port, RIGHT_Pin);

      forwardButtonState = HAL_GPIO_ReadPin(FORWARD_Port, FORWARD_Pin);

      if (rightButtonState == GPIO_PIN_RESET)
      {
    	  xMotorRunning = 0;
    	  yMotorRunning = 0;
          break;
      }

      if (leftButtonState == GPIO_PIN_RESET)
      {
    	  HAL_GPIO_WritePin(YDIR_Port, YDIR_Pin, GPIO_PIN_SET);
    	  yMotorRunning = 1;
      }
      else
      {
    	  yMotorRunning = 0;
    	  HAL_GPIO_WritePin(YPUL_Port, YPUL_Pin, GPIO_PIN_RESET);
      }

      if (forwardButtonState == GPIO_PIN_RESET)
      {
          // move X forward
    	  HAL_GPIO_WritePin(XDIR_Port, XDIR_Pin, GPIO_PIN_SET);
    	  xMotorRunning = 1;
      }
      else
      {
    	  xMotorRunning = 0;
    	  HAL_GPIO_WritePin(XPUL_Port, XPUL_Pin, GPIO_PIN_RESET);
      }
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {

	 //Calibrate the sensors
	 Calibration();


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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 63;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 499;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END TIM3_Init 2 */

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
void Calibration()
{
	switch (calState)
	{
	    case CAL_Z_DOWN:
	    	HAL_GPIO_WritePin(ZDIR_Port, ZDIR_Pin, GPIO_PIN_RESET);
	    	zMotorRunning  = 1;
	    	if (zProximityDetected)
	    	{
	    		calState = CAL_Z_UP;
	    	}
	        break;

	    case CAL_Z_UP:
	    	//Incremented in timer interrupt
	    	zPulseCount = 0;

	    	//Up direction
	    	HAL_GPIO_WritePin(ZDIR_Port, ZDIR_Pin, GPIO_PIN_SET);

	    	zMotorRunning = 1;

	        // move exactly 10 mm, 50 pulses per mm so 500 pulses per 10mm
	    	if (zPulseCount >= 500)
	    	{
	    	    zMotorRunning = 0;

	    	    HAL_GPIO_WritePin(ZPUL_Port, ZPUL_Pin, GPIO_PIN_RESET);

	    	    zPosition = 0;

	    	    calState = CAL_Y_RIGHT;
	    	}
	        break;
	        //Move right until D9 interrupt
	    case CAL_Y_RIGHT:
	    	//Sets y-direction to right
	    	HAL_GPIO_WritePin(YDIR_Port, YDIR_Pin, GPIO_PIN_RESET);
	    	//Turns on y motor (timer interrupt checks this)
	    	yMotorRunning = 1;
	    	if (yProximityDetected)
	    	{
	    		calState = CAL_Y_LEFT;
	    	}
	        break;
	    //Move 100 mm left
	    case CAL_Y_LEFT:
	        yPulseCount = 0;

	        HAL_GPIO_WritePin(YDIR_Port, YDIR_Pin, GPIO_PIN_SET);
	        if (yPulseCount >= 5000)
	        {
	        	yMotorRunning = 0;

	        	HAL_GPIO_WritePin(YPUL_Port, ZPUL_Pin, GPIO_PIN_RESET);

	        	yPosition = 0;

	        	calState = CAL_X_BACK;
	        }
	        break;

	    case CAL_X_BACK:
	        // move backward until D10 interrupt
	    	HAL_GPIO_WritePin(XDIR_Port, XDIR_Pin, GPIO_PIN_RESET);

	    	xMotorRunning = 1;
	    	if (xProximityDetected = 1)
	    	{
	    		calState = CAL_X_FORWARD;
	    	}
	        break;

	    case CAL_X_FORWARD:
	        // move measured distance
	    	HAL_GPIO_WritePin(XDIR_Port, XDIR_Pin, GPIO_PIN_SET);

	    	if (xPulseCount >= 5000)
	    	{
	    		xMotorRunning = 0;

	    		HAL_GPIO_WritePin(XPUL_Port, XDIR_Pin, GPIO_PIN_RESET);

	    		calState = CAL_WAIT;
	    	}

	        break;

	    case CAL_WAIT:
	        // wait 5 seconds
	        break;

	    case CAL_DONE:
	        break;
	}
}


//Interrupt triggered externally
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
	UNUSED(GPIO_Pin);

	// Z-axis proximity pin triggered
	if (GPIO_Pin == ZPROX_Pin)
	{
		zProxmityDetected = 1;
		zMotorRunning = 0;
	    HAL_GPIO_WritePin(ZPUL_Port, ZPUL_Pin, GPIO_PIN_RESET);

	}
	//Symmetrical
	else if (GPIO_Pin == YPROX_Pin)
	{
		yProximityDetected = 1;

		yMotorRunning = 0;

		HAL_GPIO_WritePin(YPUL_Port, ZPUL_Pin, GPIO_PIN_RESET);
	}
	else if (GPIO_Pin == XPROX_Pin)
	{
		xProximityDetected = 1;
		xMotorRunning = 0;
		HAL_GPIO_WritePin(XPUL_Port, XPUL_Pin, GPIO_PIN_RESET);
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
	if (htim->Instance == TIM2)
	{
	    if (zMotorRunning)
	    {
	        zStepState = !zStepState;

	        HAL_GPIO_WritePin(ZPUL_Port, ZPUL_Pin, zStepState ? GPIO_PIN_SET : GPIO_PIN_RESET);
	        if (zStepState == 1)
	        {
	            zPulseCount++;
	        }
	    }

	    if (yMotorRunning)
	    {
	        yStepState = !yStepState;

	        HAL_GPIO_WritePin(YPUL_Port, YPUL_Pin, yStepState ? GPIO_PIN_SET : GPIO_PIN_RESET);
	    }

	    if (xMotorRunning)
	    {
	        xStepState = !xStepState;

	        HAL_GPIO_WritePin(XPUL_Port, XPUL_Pin, xStepState ? GPIO_PIN_SET : GPIO_PIN_RESET);
	    }
	}
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
