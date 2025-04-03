/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "limits.h"
#include "string.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MIN_BUFFER_SIZE 0
#define MAX_BUFFER_SIZE SCHAR_MAX

#define CHAR_DEFAULT_SPACE 0x20
#define CHAR_ESCAPE_BACKSLASH 0x08
#define CHAR_ESCAPE_CARRIAGE_RETURN 0x0D
#define CHAR_SPECIAL_NULL_TERMINATOR 0x00
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static const uint8_t CHAR_BACKSLASH = CHAR_ESCAPE_BACKSLASH;
static const uint8_t CHAR_NULL_TERMINATOR = CHAR_SPECIAL_NULL_TERMINATOR;

static uint8_t uartInputBufferData;
static uint8_t uartInputBufferIndex;
static char uartInputBuffer[MAX_BUFFER_SIZE];

static uint8_t uartOutputBufferIndex;
static char uartOutputBuffer[MAX_BUFFER_SIZE];

static const char* const COMMAND_STOP_PWM = "stop pwm";
static const char* const MESSAGE_SUCCESS_STOP_PWM = "[LOG] PWM stopped\r\n";

static const char* const COMMAND_START_PWM = "start pwm";
static const char* const MESSAGE_SUCCESS_START_PWM = "[LOG] PWM started with duty cycle %lu%\%\r\n";

static const char* const COMMAND_STOP_TIMER = "stop timer";
static const char* const MESSAGE_SUCCESS_STOP_TIMER = "[LOG] Timer stopped\r\n";

static const char* const COMMAND_START_TIMER = "start timer";
static const char* const MESSAGE_SUCCESS_START_TIMER = "[LOG] Timer started at %lu ms\r\n";

static const char* const COMMAND_PRINT_VALUE_ADC = "print adc";
static const char* const MESSAGE_SUCCESS_PRINT_VALUE_ADC = "[LOG] ADC = %lu\r\n";

static const char* const COMMAND_TURN_LED_ON = "led on";
static const char* const MESSAGE_SUCCESS_TURN_LED_ON = "[LOG] Turned LED ON\r\n";

static const char* const COMMAND_TURN_LED_OFF = "led off";
static const char* const MESSAGE_SUCCESS_TURN_LED_OFF = "[LOG] Turned LED OFF\r\n";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
static void resetBuffers(void);
static void processCommand(void);
static void processCommandStopPWM(void);
static void processCommandStartPWM(void);
static void processCommandStopTimer(void);
static void processCommandStartTimer(void);
static void processCommandTurnLedOn(void);
static void processCommandTurnLedOff(void);
static void processCommandPrintValueADC(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6);

    HAL_TIM_Base_Start_IT(&htim3);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance != USART1)
    	return;

    switch (uartInputBufferData) {
        case CHAR_ESCAPE_BACKSLASH:
            if (uartInputBufferIndex > MIN_BUFFER_SIZE)
                uartInputBuffer[--uartInputBufferIndex] = CHAR_NULL_TERMINATOR;
            break;
        case CHAR_ESCAPE_CARRIAGE_RETURN:
            uartInputBuffer[uartInputBufferIndex++] = CHAR_NULL_TERMINATOR;
            processCommand();
            resetBuffers();
            break;
        default:
        	if (uartInputBufferIndex < MAX_BUFFER_SIZE)
        		uartInputBuffer[uartInputBufferIndex++] = uartInputBufferData;
            break;
    }

    HAL_UART_Receive_IT(&huart1, &uartInputBufferData, 1);
}

static void resetBuffers(void) {
	memset(uartInputBuffer, 0, MAX_BUFFER_SIZE);
	memset(uartOutputBuffer, 0, MAX_BUFFER_SIZE);
    uartInputBufferIndex = uartOutputBufferIndex = 0;
}

static void processCommand(void) {
	const int COMPARISON_EQUALITY = 0;

	if (strncmp(uartInputBuffer, COMMAND_STOP_PWM, strlen(COMMAND_STOP_PWM)) == COMPARISON_EQUALITY) {
		processCommandStopPWM();
		return;
	}

	if (strncmp(uartInputBuffer, COMMAND_START_PWM, strlen(COMMAND_START_PWM)) == COMPARISON_EQUALITY) {
		processCommandStartPWM();
		return;
	}

	if (strncmp(uartInputBuffer, COMMAND_STOP_TIMER, strlen(COMMAND_STOP_TIMER)) == COMPARISON_EQUALITY) {
		processCommandStopTimer();
		return;
	}

	if (strncmp(uartInputBuffer, COMMAND_START_TIMER, strlen(COMMAND_START_TIMER)) == COMPARISON_EQUALITY) {
		processCommandStartTimer();
		return;
	}

	if (strncmp(uartInputBuffer, COMMAND_TURN_LED_ON, strlen(COMMAND_TURN_LED_ON)) == COMPARISON_EQUALITY) {
		processCommandTurnLedOn();
		return;
	}

	if (strncmp(uartInputBuffer, COMMAND_TURN_LED_OFF, strlen(COMMAND_TURN_LED_OFF)) == COMPARISON_EQUALITY) {
		processCommandTurnLedOff();
		return;
	}

	if (strncmp(uartInputBuffer, COMMAND_PRINT_VALUE_ADC, strlen(COMMAND_PRINT_VALUE_ADC)) == COMPARISON_EQUALITY) {
		processCommandPrintValueADC();
		return;
	}

    sprintf(uartOutputBuffer, "[ERROR] Unknown command\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)uartOutputBuffer, strlen(uartOutputBuffer), HAL_MAX_DELAY);
}

static void processCommandStopPWM(void) {
    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
    HAL_UART_Transmit(&huart2, (uint8_t*)MESSAGE_SUCCESS_STOP_PWM, strlen(MESSAGE_SUCCESS_STOP_PWM), HAL_MAX_DELAY);
}

static void processCommandStartPWM(void) {
    const char* duty_str = uartInputBuffer + strlen(COMMAND_START_PWM);

    while (*duty_str == CHAR_DEFAULT_SPACE)
        ++duty_str;

    if (*duty_str == CHAR_NULL_TERMINATOR) {
        sprintf(uartOutputBuffer, "[ERROR] No PWM duty cycle specified\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)uartOutputBuffer, strlen(uartOutputBuffer), HAL_MAX_DELAY);
        return;
    }

    uint32_t duty_percent = atoi(duty_str);

    if (duty_percent > 100) {
        sprintf(uartOutputBuffer, "[ERROR] Invalid PWM duty cycle (0-100%%): %s\r\n", duty_str);
        HAL_UART_Transmit(&huart2, (uint8_t*)uartOutputBuffer, strlen(uartOutputBuffer), HAL_MAX_DELAY);
        return;
    }

    uint32_t arr = htim3.Instance->ARR;
    uint32_t pulse = (arr * duty_percent) / 100;

    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse);

    sprintf(uartOutputBuffer, MESSAGE_SUCCESS_START_PWM, duty_percent);
    HAL_UART_Transmit(&huart2, (uint8_t*)uartOutputBuffer, strlen(uartOutputBuffer), HAL_MAX_DELAY);
}

static void processCommandStopTimer(void) {
    HAL_TIM_Base_Stop_IT(&htim3);
    __HAL_TIM_DISABLE(&htim3);

    HAL_UART_Transmit(&huart2, (uint8_t*)MESSAGE_SUCCESS_STOP_TIMER, strlen(MESSAGE_SUCCESS_STOP_TIMER), HAL_MAX_DELAY);
}

static void processCommandStartTimer(void) {
	const char* period_str = uartInputBuffer + strlen(COMMAND_START_TIMER);

	while (*period_str == CHAR_DEFAULT_SPACE)
		++period_str;

	if (*period_str == CHAR_NULL_TERMINATOR) {
		sprintf(uartOutputBuffer, "[ERROR] No timer period specified\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)uartOutputBuffer, strlen(uartOutputBuffer), HAL_MAX_DELAY);
		return;
	}

	uint32_t milliseconds = atoi(period_str);

	if (milliseconds == 0) {
		sprintf(uartOutputBuffer, "[ERROR] Invalid timer period: %s\r\n", period_str);
		HAL_UART_Transmit(&huart2, (uint8_t*)uartOutputBuffer, strlen(uartOutputBuffer), HAL_MAX_DELAY);
		return;
	}

	__HAL_TIM_DISABLE(&htim3);

	uint32_t prescaler = htim3.Instance->PSC;
	uint32_t timerClock = HAL_RCC_GetHCLKFreq() / (prescaler + 1);
	uint32_t arr = (timerClock * milliseconds) / 1000 - 1;

	if (arr > UINT16_MAX) {
		sprintf(uartOutputBuffer, "[ERROR] Timer period too large (max ~8192 ms with current prescaler)\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)uartOutputBuffer, strlen(uartOutputBuffer), HAL_MAX_DELAY);
		return;
	}

	htim3.Instance->ARR = arr;
	htim3.Instance->EGR = TIM_EGR_UG;

	__HAL_TIM_ENABLE(&htim3);
	HAL_TIM_Base_Start_IT(&htim3);

	sprintf(uartOutputBuffer, MESSAGE_SUCCESS_START_TIMER, milliseconds);
	HAL_UART_Transmit(&huart2, (uint8_t*)uartOutputBuffer, strlen(uartOutputBuffer), HAL_MAX_DELAY);
}

static void processCommandTurnLedOn(void) {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	HAL_UART_Transmit(&huart2, (uint8_t*)MESSAGE_SUCCESS_TURN_LED_ON, strlen(MESSAGE_SUCCESS_TURN_LED_ON), HAL_MAX_DELAY);
}

static void processCommandTurnLedOff(void) {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	HAL_UART_Transmit(&huart2, (uint8_t*)MESSAGE_SUCCESS_TURN_LED_OFF, strlen(MESSAGE_SUCCESS_TURN_LED_OFF), HAL_MAX_DELAY);
}

static void processCommandPrintValueADC(void) {
    sprintf(uartOutputBuffer, MESSAGE_SUCCESS_PRINT_VALUE_ADC, HAL_ADC_GetValue(&hadc1));
    HAL_UART_Transmit(&huart2, (uint8_t*)uartOutputBuffer, strlen(uartOutputBuffer), HAL_MAX_DELAY);
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
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);

  HAL_UART_Receive_IT(&huart1, &uartInputBufferData, 1);

  __HAL_RCC_ADC1_CLK_ENABLE();
  HAL_ADC_Start(&hadc1);

  HAL_UART_Transmit(&huart1, &CHAR_BACKSLASH, 1, HAL_MAX_DELAY);
  HAL_UART_Transmit(&huart2, &CHAR_BACKSLASH, 1, HAL_MAX_DELAY);
  /* USER CODE END 2 */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim3.Init.Prescaler = 7999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
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
  sConfigOC.Pulse = 500;
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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 9600;
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
