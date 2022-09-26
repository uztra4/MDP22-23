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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "pid.h"
#include "ICM20948.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STRAIGHT 150
#define LEFT 100
#define RIGHT 270 //MAX IS 270


/* Controller parameters */
#define PID_KP  1.0f // default 2.0f
#define PID_KI  0.0f // default 0.5f

#define PID_LIM_MIN  900 //
#define PID_LIM_MAX  3600 //

#define PID_TURNING_LIM_MIN  2200 //
#define PID_TURNING_LIM_MAX  3000 //

#define SAMPLE_TIME_S 0.05f

#define STRAIGHTRATIOF 0.9694f //0.702
#define STRAIGHTRATIOR 1.07705f //1.078 abit l. 1.07705 still left
#define DISTANCE_ERROR_OFFSETF 0.01f
#define DISTANCE_ERROR_OFFSETR 0.06f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for OLEDTask */
osThreadId_t OLEDTaskHandle;
const osThreadAttr_t OLEDTask_attributes = {
  .name = "OLEDTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for GyroRead */
osThreadId_t GyroReadHandle;
const osThreadAttr_t GyroRead_attributes = {
  .name = "GyroRead",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ServoMotor */
osThreadId_t ServoMotorHandle;
const osThreadAttr_t ServoMotor_attributes = {
  .name = "ServoMotor",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
ICM20948 imu; // Declare a variable with structure type ICM20948
uint8_t moving;
float gyroSum;
uint8_t aRxBuffer[1];
uint8_t newCmdReceived;
uint8_t cmd;
uint32_t data;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void *argument);
void oled_show(void *argument);
void gyro_read(void *argument);
void servoMotor(void *argument);

/* USER CODE BEGIN PFP */
void sendToRPI(char* msg);
void move(float distance, int forward);
void turn(uint8_t direction, uint8_t forward);


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

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM8_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of OLEDTask */
//  OLEDTaskHandle = osThreadNew(oled_show, NULL, &OLEDTask_attributes);

  /* creation of GyroRead */
  GyroReadHandle = osThreadNew(gyro_read, NULL, &GyroRead_attributes);

  /* creation of ServoMotor */
//  ServoMotorHandle = osThreadNew(servoMotor, NULL, &ServoMotor_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 160;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
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
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
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
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 7199;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : OLED_SCL_Pin OLED_SDA_Pin OLED_RST_Pin OLED_DC_Pin
                           LED3_Pin */
  GPIO_InitStruct.Pin = OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : AIN2_Pin AIN1_Pin BIN1_Pin BIN2_Pin */
  GPIO_InitStruct.Pin = AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
// For own reference, this function seems to be called after
// interrupt from the HAL_UART_Receive_IT function
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//Prevent unused argument(s) compilation warning
	UNUSED(huart);
	static i = 0; //static means it doesn't get erase after every loop

	if (aRxBuffer[0] == 'n'){
		i = 0;
		newCmdReceived = 1;
	}
	else if (i==0)
	{
		cmd = aRxBuffer[0];
		i++;
	}
	else if (i > 0)
	{
		data = data*10 + (aRxBuffer[0] - '0');
	}

//	uint8_t message[20];
	HAL_UART_Receive_IT(&huart3,(uint8_t *) aRxBuffer, 1);
	HAL_UART_Transmit(&huart3, (uint8_t *) aRxBuffer, 1, 0xFFFF);
//	sprintf(message, "%c %d %c %d\0", aRxBuffer[0], newCmdReceived, cmd, data);
//	OLED_ShowString(10, 40, message);
//	OLED_Refresh_Gram(); //Refresh Ram
}

/**
  * @brief send msg to RPI through UART after execute command
  * @param msg[] for the message to be sent
  * @retval None
  */
void sendToRPI(char* msg)
{
	HAL_UART_Transmit(&huart3,(uint8_t *) msg,strlen(msg),0xFFFF);
}

void move(float distance, int forward)
{
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	htim1.Instance -> CCR4 = STRAIGHT; //centre
	osDelay(100);

	// init PID controller (left)
	PIDController pidLeft = { PID_KP, PID_KI,
						  PID_LIM_MIN, PID_LIM_MAX,
						  SAMPLE_TIME_S };


	// init PID controller (right)
	PIDController pidRight = { PID_KP, PID_KI,
						  PID_LIM_MIN, PID_LIM_MAX,
						  SAMPLE_TIME_S };

	PIDController_Init(&pidLeft);
	PIDController_Init(&pidRight);

	// wheel information
	float wheel_rotationTicksL = 779; // for back left wheel, around 1550-1600
	float wheel_rotationTicksR = 1561; // for back right wheel

	float wheel_circumference = 21.8f; // NEED TO MEASURE AND CHANGE

	// start encoder
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); //encoderA
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); //encoderB

	// wheel ticks
	int leftTick_prev = __HAL_TIM_GET_COUNTER(&htim2);
	int rightTick_prev = __HAL_TIM_GET_COUNTER(&htim3);

	// cpu ticks
	uint32_t startTime = HAL_GetTick();
	uint32_t prevTime = startTime;
	uint32_t currTime;

	// distance checkers for both wheels
	float totalDistance_left = 0;
	float totalDistance_right = 0;

	// left measured distance
	float distLeft;

	// right measured distance
	float distRight;

	int leftTick;
	int rightTick;

	int diffLeft = 0;
	int diffRight = 0;

	float distanceError;

	//pwm values
	uint16_t pwmValA = 2000;
	uint16_t pwmValB = 2000;

	// OLED variables for testing PID
	uint8_t messageA[20];
	uint8_t messageB[20];

	if(forward){
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
	}else{
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
	}

	/*Infinite loop*/
	for(;;)
	{

		 __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmValA);
		 __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmValB);
		 currTime = HAL_GetTick();
		 moving = 1;

		 if (currTime-prevTime > 50L)
		 {
			 leftTick = __HAL_TIM_GET_COUNTER(&htim2);
			 rightTick = __HAL_TIM_GET_COUNTER(&htim3);

			 diffLeft = 0;
			 diffRight = 0;

			 if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2))
			 {
				 if (leftTick < leftTick_prev)
					 diffLeft = leftTick_prev - leftTick;
				 else
					 diffLeft = (65535 - leftTick) + leftTick_prev;
			 }
			 else
			 {
				 if (leftTick > leftTick_prev)
					 diffLeft = leftTick - leftTick_prev;
				 else
					 diffLeft = 65535 - leftTick_prev + leftTick;
			 }

			 if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3))
			 {
				 if (rightTick < rightTick_prev)
					 diffRight = rightTick_prev - rightTick;
				 else
					 diffRight = (65535 - rightTick) + rightTick_prev;
			 }
			 else
			 {
				 if (rightTick > rightTick_prev)
					 diffRight = rightTick - rightTick_prev;
				 else
					 diffRight = 65535 - rightTick_prev + rightTick;
			 }

			 // left measured distance
			 distLeft = ((float)diffLeft/wheel_rotationTicksL) * wheel_circumference;
			 totalDistance_left += distLeft;

			 // right measured distance
			 distRight = ((float)diffRight/wheel_rotationTicksR) * wheel_circumference;
			 totalDistance_right += distRight;

			 // for straight ratio, higher means right dist decrease.
			 // 1.015 veer right, 1.005 veer left, 1.010 ABIT right, 1.008 almost perfect.


			 if (forward){

				 pwmValA = PIDController_Update(&pidLeft, totalDistance_right*STRAIGHTRATIOF, totalDistance_left, pwmValA);

				 pwmValB = PIDController_Update(&pidRight, totalDistance_left, totalDistance_right*STRAIGHTRATIOF, pwmValB);
			 }else{
				 pwmValA = PIDController_Update(&pidLeft, totalDistance_right, totalDistance_left*STRAIGHTRATIOR, pwmValA);

				 pwmValB = PIDController_Update(&pidRight, totalDistance_left*STRAIGHTRATIOR, totalDistance_right, pwmValB);
			 }

			 if(forward){
				 distanceError = DISTANCE_ERROR_OFFSETF * distance;
			 }else{
				 distanceError = DISTANCE_ERROR_OFFSETR * distance;
			 }

			 if (totalDistance_left >= (distance+distanceError) || totalDistance_right >= (distance+distanceError))
			 {
				 __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);
				 __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0);
				 moving = 0;
					sprintf(messageA, "SLeft: %2d\0", pwmValA);
					OLED_ShowString(10, 20, messageA);
					sprintf(messageA, "TLeft: %.2f\0", totalDistance_left);
					OLED_ShowString(10, 30, messageA);

//					sprintf(messageB, "SRight: %2d\0", pwmValB);
//					OLED_ShowString(10, 40, messageB);
//					sprintf(messageB, "TRight: %.2f\0", totalDistance_right);
//					OLED_ShowString(10, 50, messageB);

					sprintf(messageA, "%5.2f\0", gyroSum);
					OLED_ShowString(10, 40, messageA);
					OLED_Refresh_Gram(); //Refresh Ram
				 break;
			 }



//			// OLED
//			sprintf(messageA, "SLeft: %2d\0", diffLeft);
//			OLED_ShowString(10, 20, messageA);
//			sprintf(messageA, "TLeft: %.2f\0", totalDistance_left);
//			OLED_ShowString(10, 30, messageA);

//			sprintf(messageB, "SRight: %2d\0", diffRight);
//			OLED_ShowString(10, 40, messageB);
//			sprintf(messageB, "TRight: %.2f\0", totalDistance_right);
//			OLED_ShowString(10, 50, messageB);

			prevTime = currTime;
			leftTick_prev = leftTick;
			rightTick_prev = rightTick;
		 }
	}
	moving = 0;
	osDelay(1000);
	return;
}

void reverse(float distance)
{
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	htim1.Instance -> CCR4 = STRAIGHT; //centre
	osDelay(100);

	// wheel information
	float wheel_rotationTicksL = 779; // for back left wheel, around 1550-1600
	float wheel_rotationTicksR = 1561; // for back right wheel

	float wheel_circumference = 21.8f; // NEED TO MEASURE AND CHANGE

	// start encoder
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); //encoderA
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); //encoderB

	// wheel ticks
	int leftTick_prev = __HAL_TIM_GET_COUNTER(&htim2);
	int rightTick_prev = __HAL_TIM_GET_COUNTER(&htim3);

	// cpu ticks
	uint32_t startTime = HAL_GetTick();
	uint32_t prevTime = startTime;
	uint32_t currTime;

	// distance checkers for both wheels
	float totalDistance_left = 0;
	float totalDistance_right = 0;

	// left measured distance
	float distLeft;

	// right measured distance
	float distRight;

	int leftTick;
	int rightTick;

	int diffLeft = 0;
	int diffRight = 0;

	float distanceError;

	//pwm values
	uint16_t pwmValA = 2400;
	uint16_t pwmValB = 2400;

	// OLED variables for testing PID
	uint8_t messageA[20];
	uint8_t messageB[20];


	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);

	/*Infinite loop*/
	for(;;)
	{

		 __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmValA);
		 __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmValB);
		 currTime = HAL_GetTick();
		 moving = 1;

		 if (currTime-prevTime > 50L)
		 {
			 leftTick = __HAL_TIM_GET_COUNTER(&htim2);
			 rightTick = __HAL_TIM_GET_COUNTER(&htim3);

			 diffLeft = 0;
			 diffRight = 0;

			 if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2))
			 {
				 if (leftTick < leftTick_prev)
					 diffLeft = leftTick_prev - leftTick;
				 else
					 diffLeft = (65535 - leftTick) + leftTick_prev;
			 }
			 else
			 {
				 if (leftTick > leftTick_prev)
					 diffLeft = leftTick - leftTick_prev;
				 else
					 diffLeft = 65535 - leftTick_prev + leftTick;
			 }

			 if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3))
			 {
				 if (rightTick < rightTick_prev)
					 diffRight = rightTick_prev - rightTick;
				 else
					 diffRight = (65535 - rightTick) + rightTick_prev;
			 }
			 else
			 {
				 if (rightTick > rightTick_prev)
					 diffRight = rightTick - rightTick_prev;
				 else
					 diffRight = 65535 - rightTick_prev + rightTick;
			 }

			 // left measured distance
			 distLeft = ((float)diffLeft/wheel_rotationTicksL) * wheel_circumference;
			 totalDistance_left += distLeft;

			 // right measured distance
			 distRight = ((float)diffRight/wheel_rotationTicksR) * wheel_circumference;
			 totalDistance_right += distRight;

			 // for straight ratio, higher means right dist decrease.
			 // 1.015 veer right, 1.005 veer left, 1.010 ABIT right, 1.008 almost perfect.


			 pwmValA = 2400 + gyroSum * 2.7;

			 pwmValB = 2400 - gyroSum * 2.7;

			 distanceError = DISTANCE_ERROR_OFFSETR * distance;

			 if (totalDistance_left >= (distance+distanceError) || totalDistance_right >= (distance+distanceError))
			 {
				 __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);
				 __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,0);
				 moving = 0;
					sprintf(messageA, "SLeft: %2d\0", pwmValA);
					OLED_ShowString(10, 20, messageA);
					sprintf(messageA, "TLeft: %.2f\0", totalDistance_left);
					OLED_ShowString(10, 30, messageA);

					sprintf(messageB, "SRight: %2d\0", pwmValB);
					OLED_ShowString(10, 40, messageB);
					sprintf(messageB, "TRight: %.2f\0", totalDistance_right);
					OLED_ShowString(10, 50, messageB);
					OLED_Refresh_Gram(); //Refresh Ram
				 break;
			 }



//			// OLED
//			sprintf(messageA, "SLeft: %2d\0", diffLeft);
//			OLED_ShowString(10, 20, messageA);
//			sprintf(messageA, "TLeft: %.2f\0", totalDistance_left);
//			OLED_ShowString(10, 30, messageA);

//			sprintf(messageB, "SRight: %2d\0", diffRight);
//			OLED_ShowString(10, 40, messageB);
//			sprintf(messageB, "TRight: %.2f\0", totalDistance_right);
//			OLED_ShowString(10, 50, messageB);

			prevTime = currTime;
			leftTick_prev = leftTick;
			rightTick_prev = rightTick;
		 }
	}
	moving = 0;
	osDelay(1000);
	return;
}


//using this to slowly get hold of the trend for the no. of degree vs gyro values
//mainly for task A4, require to take in angles from 90 to 360

//////////////////////Collecting data for data and angle///////////////////////
//preferable turning radius = 20cm
//turn to reach 90 degrees w.r.t turining right
//90 degrees: 8500
//120 degrees:
//150 degrees:
//180 degrees: 18050
//210 degrees:
//240 degrees:
//270 degrees:27000
//300 degrees:
//330 degrees:
//360 degrees:36500
///////////////////////////////////////////////////////////////////////////
void turn(uint8_t direction, uint8_t forward)
{
	float gyro;
//	uint8_t angle;
//	angle = data*1000; //relationship of degree angle to gyro values
	uint8_t GYROSUM[20];
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	//move forward
	if (forward)
	{
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);

		if (direction)
		{
			//left
			htim1.Instance -> CCR4 = LEFT;
			osDelay(100);
			 __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 2000);
			 __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2, 2000);
			 moving = 1;
			 while (moving)
			 {
				 if (fabs(gyroSum) < data)
				 {
					 //continue moving
					 osDelay(1);
				 }
				 else
				 {
					 break;
				 }

			 }
		}
		else
		{
			//right
			htim1.Instance -> CCR4 = RIGHT;
			osDelay(100);
			 __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 1500);
			 __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2, 2000);
			 moving = 1;
			 while (moving)
			 {
				 if (fabs(gyroSum) < data)
				 {
					 //continue moving
					 osDelay(1);
				 }
				 else
				 {
					 break;
				 }

			 }
		}

	}
	//backwards
	else
	{
		//left
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
		if (direction)
		{
			htim1.Instance -> CCR4 = LEFT;
			 __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 2000);
			 __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2, 2000);
			 moving = 1;
			 while (moving)
			 {
				 if (fabs(gyroSum) < data)
				 {
					 //continue moving
					 osDelay(1);
				 }
				 else
				 {
					 break;
				 }

			 }
		}
		//right
		else
		{
			htim1.Instance -> CCR4 = RIGHT; //right
			 __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 2000);
			 __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2, 2000);
			 moving = 1;
			 while (moving)
			 {
				 if (fabs(gyroSum) < data)
				 {
					 //continue moving
					 osDelay(1);
				 }
				 else
				 {
					 break;
				 }

			 }
		}
	}
	//Print GyroSum on OLED
	sprintf(GYROSUM, "%5.2f\0", gyroSum);
	OLED_ShowString(10, 10, GYROSUM);
	OLED_Refresh_Gram(); //Refresh Ram

	//Stop wheels and let wheels be straight
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2, 0);
	moving = 0;
	htim1.Instance -> CCR4 = STRAIGHT; //centre

	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);

	move(1, 0); //to let robot stop immediately as robot moves forward due to momentum
	osDelay(1000);
	return;
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	uint8_t instrBuffer[20], angle;
	uint16_t i = 0;
	HAL_UART_Receive_IT(&huart3,(uint8_t *) aRxBuffer, 1);
	moving = 0;
	cmd = data = newCmdReceived = 0;
	uint8_t messageA[20];

  /* Infinite loop */
  for(;;)
  {

	  //Toggle LED just to see if the code is running
	HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);



	// assume
	// 1: forward
	// 2: left
	// 3: right
	// 4: reverse
	// 5: shortest path
	if (newCmdReceived == 1)
	{
		newCmdReceived = 0;
		switch(cmd)
		{
		case 'w':
			move(data, 1);
			sendToRPI("Forward done!\n\0");
			break;
		case 'l':
			turn(1, 1);
			sendToRPI("Left done!\0");
			break;
		case 'r':
			turn(0, 1);
			sendToRPI("Right done!\0");
			break;
		case 's':
			move(data, 0);
			sendToRPI("Reverse done!\0");
			break;
		case 'L':
			turn(1, 0);
			sendToRPI("Reverse Left done!\0");
			break;
		case 'R':
			turn(0, 0);
			sendToRPI("Reverse Right done!\0");
			break;
		case 'p': // take photo
			sendToRPI("RPI:s"); // command for rpi to take photo
			break;
		default:
			break;
		}
		data = 0;
	}
	osDelay(1000);
  }

  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_oled_show */
/**
* @brief Function implementing the OLEDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_oled_show */
void oled_show(void *argument)
{
  /* USER CODE BEGIN oled_show */
  uint8_t message[20] = "Hello EuChing!\0";
  /* Infinite loop */
  for(;;)
  {
	sprintf(message, "%s\0", aRxBuffer);
	OLED_ShowString(10, 10, message);
	OLED_Refresh_Gram(); //Refresh Ram
	osDelay(1000);
  }
  /* USER CODE END oled_show */
}

/* USER CODE BEGIN Header_gyro_read */
/**
* @brief Function implementing the myTask09 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_gyro_read */
void gyro_read(void *argument)
{
  /* USER CODE BEGIN gyro_read */
  /* Infinite loop */
	uint8_t status = IMU_Initialise(&imu, &hi2c1, &huart3);

	uint8_t gbuf[50];
	float gyro;
  for(;;)
  {
	//to sum up gyro values when moving to compare during turn
	if (moving)
	{
		IMU_GyroRead(&imu);
		gyro = imu.gyro[2];
		gyroSum += fabs(gyro);
//		sprintf(gbuf, "[DEBUG]\n\rGYRO\n\rZ = %5.2f\n\rTotal\n\r = %5.2f\n\r", gyro, gyroSum); // Get Z value of Gyroscope
//		HAL_UART_Transmit(&huart3, gbuf, sizeof(gbuf)-1, HAL_MAX_DELAY);
		osDelay(10);
	}
	//set gyroSum to 0 when not moving
	else
	{
		gyroSum = 0;
//		sprintf(gbuf, "[DEBUG] Not Moving!\n\r Total\n\r = %5.2f\n\r", gyroSum); // Get Z value of Gyroscope
//		HAL_UART_Transmit(&huart3, gbuf, sizeof(gbuf)-1, HAL_MAX_DELAY);
		osDelay(10);
	}
  }
  /* USER CODE END gyro_read */
}

/* USER CODE BEGIN Header_servoMotor */
/**
* @brief Function implementing the ServoMotor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_servoMotor */
void servoMotor(void *argument)
{
  /* USER CODE BEGIN servoMotor */
	// start encoder
		HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); //encoderA
		HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); //encoderB

		// wheel ticks
		int leftTick_prev = __HAL_TIM_GET_COUNTER(&htim2);
		int rightTick_prev = __HAL_TIM_GET_COUNTER(&htim3);

		// cpu ticks
		uint32_t startTime = HAL_GetTick();
		uint32_t prevTime = startTime;
		uint32_t currTime;


		int leftTick;
		int rightTick;

		int diffLeft = 0;
		int diffRight = 0;

		float gyro;

		// OLED variables for testing PID
		uint8_t messageA[20];
		uint8_t messageB[20];


		/*Infinite loop*/
		for(;;)
		{
			 currTime = HAL_GetTick();

			 if (currTime-prevTime > 50L)
			 {
				 leftTick = __HAL_TIM_GET_COUNTER(&htim2);
				 rightTick = __HAL_TIM_GET_COUNTER(&htim3);

				 if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2))
				 {
//					 diffLeft += leftTick_prev - leftTick;
//				 }
//				 else
//				 {
//					 diffLeft += leftTick - leftTick_prev;
//				 }
				 if (leftTick < leftTick_prev)
						 diffLeft += leftTick_prev - leftTick;
				 }
				 else
				 {
					 if (leftTick > leftTick_prev)
						 diffLeft += leftTick - leftTick_prev;
				 }

				 if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3))
				 {
//					 diffRight += rightTick_prev - rightTick;
//				 }
//				 else
//				 {
//					 diffRight += rightTick - rightTick_prev;
//				 }
				 if (rightTick < rightTick_prev)
						 diffRight += rightTick_prev - rightTick;
				 }
				 else
				 {
					 if (rightTick > rightTick_prev)
						 diffRight += rightTick - rightTick_prev;
				 }



	//			// OLED
				sprintf(messageA, "SLeft: %2d\0", diffLeft);
				OLED_ShowString(10, 20, messageA);

				IMU_GyroRead(&imu);
				gyro = imu.gyro[2];
				sprintf(messageA, "GyroZ: %2d\0", gyro);
				OLED_ShowString(10, 30, messageA);

				sprintf(messageB, "SRight: %2d\0", diffRight);
				OLED_ShowString(10, 40, messageB);
				OLED_Refresh_Gram(); //Refresh Ram

				prevTime = currTime;
				leftTick_prev = leftTick;
				rightTick_prev = rightTick;
			 }
		}
  /* USER CODE END servoMotor */
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
