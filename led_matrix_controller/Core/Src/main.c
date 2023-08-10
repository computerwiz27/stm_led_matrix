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
#include "serial_interface.h"

#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_LED 2

const uint8_t MATRIX_SHAPE[3] = {16, 4, 4}; // height, width, deptth
const uint8_t TOWERS_SHAPE[2] = {1, 1};     // rows, aisles

// LED data transfer times
#define DUTY_CYCLE 1250 // ns
#define T0H 280         // ns
#define T1H 650         // ns
#define RES 140         // us
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_tim1_ch1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
int quitFlag = 0;

SERIAL_INTERFACE ser;

uint8_t LED_Data[MAX_LED][3];
int dataSentFlag = 0;

// data values
int one;
int zero;
int reset;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void SerialInterface_Init(void);
void SerialInterface_Free(void);
void WaitForConnection(void);
void HAL_TIM_PulseFinishedCallback(TIM_HandleTypeDef);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef*);
void Transmit(uint8_t*, uint16_t);
int Transmit_WaitForConfirm(uint8_t*, uint16_t);
void ClearTXBuffer();
void ClearRXBuffer(UART_HandleTypeDef *);
void ProcessCommand(uint8_t *);
void ProcessData(uint8_t *);
void Blink(int);
void SendSpecs(void);
void CalculateDataValues(void);
void PWMSend_LEDs(void);
void Set_LED(uint16_t, uint8_t, uint8_t, uint8_t);
void Set_LEDs(uint8_t, uint8_t, uint8_t);
void Reset_LEDs(void);
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  CalculateDataValues();
  SerialInterface_Init();

  HAL_UART_Receive_DMA(&huart2, ser.rx_command_buf, ser.command_size);

  WaitForConnection();
  Blink(3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (!quitFlag)
  {
    PWMSend_LEDs();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }

  HAL_UART_DMAStop(&huart2);
  SerialInterface_Free();

  return 0;
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 90-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

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
void SerialInterface_Init()
{
  ser.connected_flag = RESET;
  ser.expect_cmd_flag = SET;
  ser.expect_conf_flag = RESET;
  ser.confirm_flag = RESET;

  ser.last_cmd = CONFIRM;

  ser.command_size = 6;
  uint8_t *_rx_command = (uint8_t *)malloc(ser.command_size * sizeof(uint8_t));
  ser.rx_command_buf = _rx_command;
  uint8_t *_tx_command = (uint8_t *)malloc(ser.command_size * sizeof(uint8_t));
  ser.tx_command_buf = _tx_command;

  ser.data_size = sizeof(uint8_t) * 3 * MAX_LED;
  uint8_t *_rx_data = (uint8_t *)malloc(ser.data_size);
  ser.rx_data_buf = _rx_data;
}

void SerialInterface_Free()
{
  free(ser.rx_command_buf);
  free(ser.tx_command_buf);
  free(ser.rx_data_buf);
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);
  dataSentFlag = 1;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  HAL_UART_DMAStop(huart);

  // char msg[10] = "received\n\r";
  // Transmit(msg, sizeof(msg));

  if (ser.expect_conf_flag == SET)
  {
    ser.expect_conf_flag = RESET;
    ClearRXBuffer(huart);
    HAL_UART_Receive_DMA(huart, ser.rx_command_buf, ser.command_size);
    return;
  }

  if (ser.expect_cmd_flag == SET)
  {
    ProcessCommand(ser.rx_command_buf);
  }
  else
  {
    ProcessData(ser.rx_data_buf);
  }

  ClearRXBuffer(huart);

  if (ser.last_cmd != SEND)
  {
    Transmit(ser.tx_command_buf, ser.command_size);
  }
}

int WaitForConfirm()
{
  ser.expect_conf_flag = SET;
  HAL_UART_DMAStop(&huart2);
  HAL_UART_Receive_DMA(&huart2, ser.rx_command_buf, ser.command_size);
  ClearRXBuffer(&huart2);
  while(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) != 1);

  uint8_t confirm = ser.rx_command_buf[0]; 

  ClearRXBuffer(&huart2);

  if (confirm == CONFIRM) return 1;
  return 0;
}

void Transmit(uint8_t *pData, uint16_t Size)
{
  HAL_UART_Transmit_DMA(&huart2, pData, Size);
  while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) != 1);
  ClearTXBuffer();
}

void ProcessCommand(uint8_t *rx_command)
{
  uint8_t cmd = rx_command[0];
  uint8_t r = rx_command[1];
  uint8_t g = rx_command[2];
  uint8_t b = rx_command[3];
  uint16_t led = (uint16_t)rx_command[4] << 4 | rx_command[5];

  ClearTXBuffer();
  ser.tx_command_buf[0] = CONFIRM;

  switch (cmd)
  {
  case RECEIVE:
    ser.expect_cmd_flag = RESET;
    break;

  case SEND:
    SendSpecs();
    break;

  case SET:
    Set_LED(led, r, g, b);
    break;

  case SET_ALL:
    Set_LEDs(r, g, b);
    break;

  case RESET:
    Reset_LEDs();
    break;

  case QUIT:
    quitFlag = 1;
    break;

  case CONFIRM:
    break;
  case ERROR:
    break;

  default:
    ser.tx_command_buf[0] = ERROR;
    break;
  }

  ser.last_cmd = cmd;

  if (ser.expect_cmd_flag == SET)
  {
    HAL_UART_Receive_DMA(&huart2, ser.rx_command_buf, ser.command_size);
  }
  else
  {
    HAL_UART_Receive_DMA(&huart2, ser.rx_data_buf, ser.data_size);
  }
}

void ProcessData(uint8_t *rx_data)
{
  for (uint16_t i = 0; i < MAX_LED; i++)
  {
    Set_LED(i, rx_data[i * 3], rx_data[i * 3 + 1], rx_data[i * 3 + 2]);
  }

  ser.expect_cmd_flag = SET;
  ClearTXBuffer();
  ser.tx_command_buf[0] = CONFIRM;

  HAL_UART_Receive_DMA(&huart2, ser.rx_command_buf, ser.command_size);
}

void ClearTXBuffer()
{
  for (int i = 0; i < ser.command_size; i++)
  {
    ser.tx_command_buf[i] = 0;
  }
}

void ClearRXBuffer(UART_HandleTypeDef *huart)
{
  huart->RxXferCount = 0;
  huart->RxXferSize = 0;
  huart->pRxBuffPtr = NULL;
  huart->
}

void Blink(int n)
{
  for (int i = 0; i < n * 2; i++)
  {
    HAL_GPIO_TogglePin(GPIOA, LD2_Pin);
    HAL_Delay(100);
  }
}

void WaitForConnection()
{
  int conf = WaitForConfirm();
  if (conf == 1) 
  {
    ClearTXBuffer();
    ser.tx_command_buf[0] = CONFIRM;
    Transmit(ser.tx_command_buf, ser.command_size);
    ser.connected_flag = SET;
    return;
  }
}

void SendSpecs()
{
  uint8_t tx_specs[6];
  tx_specs[0] = MATRIX_SHAPE[0]; // matrix height
  tx_specs[1] = MATRIX_SHAPE[1]; // matrix width
  tx_specs[2] = MATRIX_SHAPE[2]; // matrix depth
  tx_specs[3] = TOWERS_SHAPE[0]; // rows of towers
  tx_specs[4] = TOWERS_SHAPE[1]; // ailes of towers

  for (int i = 0; i < ser.command_size; i++)
  {
    ser.tx_command_buf[i] = tx_specs[i];
  }

  Transmit(ser.tx_command_buf, ser.command_size);
  int conf = WaitForConfirm();
  
  if (conf != 1)
  {
    SendSpecs();
  }
}

void CalculateDataValues()
{
  int HCLKfreq = HAL_RCC_GetHCLKFreq() / 1000000; // in MHz

  int ARR = 0;
  float cycledt = 1 / (float)HCLKfreq * (float)ARR * 1000; // in ns
  while (cycledt < DUTY_CYCLE)
  {
    ARR++;
    cycledt = 1 / (float)HCLKfreq * (float)ARR * 1000;
  }
  __HAL_TIM_SET_AUTORELOAD(&htim1, ARR - 1);

  one = 0;
  float onedt = ((float)one / (float)ARR) * cycledt;
  while (onedt < T1H)
  {
    one++;
    onedt = ((float)one / (float)ARR) * cycledt;
  }

  zero = 0;
  float zerodt = ((float)zero / (float)ARR) * cycledt;
  while (zerodt < T0H)
  {
    zero++;
    zerodt = ((float)zero / (float)ARR) * cycledt;
  }

  reset = 0;
  float cyclems = (float)cycledt / 1000;
  while (cyclems * reset <= RES)
  {
    reset++;
  }
}

void PWMSend_LEDs()
{
  uint16_t *pwmData = (uint16_t *)malloc(sizeof(uint16_t) * (24 * MAX_LED + reset));
  uint32_t index = 0;
  uint32_t color;

  for (int i = 0; i < MAX_LED; i++)
  {
    color = ((LED_Data[i][0] << 16) | (LED_Data[i][1] << 8) | (LED_Data[i][2]));

    for (int j = 23; j >= 0; j--)
    {
      if (color & (1 << j))
      {
        pwmData[index] = one;
      }
      else
      {
        pwmData[index] = zero;
      }

      index++;
    }
  }

  for (int i = 0; i < reset; i++)
  {
    pwmData[index] = 0;
    index++;
  }

  HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)pwmData, index);
  free(pwmData);

  while (!dataSentFlag)
    ;
  dataSentFlag = 0;
}

void Set_LED(uint16_t no, uint8_t r, uint8_t g, uint8_t b)
{
  LED_Data[no][0] = r;
  LED_Data[no][1] = g;
  LED_Data[no][2] = b;
}

void Set_LEDs(uint8_t r, uint8_t g, uint8_t b)
{
  for (int i = 0; i < MAX_LED; i++)
  {
    Set_LED(i, r, g, b);
  }
}

void Reset_LEDs()
{
  Set_LEDs(0, 0, 0);
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
