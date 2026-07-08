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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "button.h"
#include "buzzer.h"
#include "dht11.h"
#include "hcsr04.h"
#include "mpu6050.h"
#include "servo.h"
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define OVERHEAT_THRESHOLD_C   45.0f
#define NEARBY_THRESHOLD_CM    30.0f
#define OBSTACLE_THRESHOLD_CM  10.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SensorTask */
osThreadId_t SensorTaskHandle;
const osThreadAttr_t SensorTask_attributes = {
  .name = "SensorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for FaultTask */
osThreadId_t FaultTaskHandle;
const osThreadAttr_t FaultTask_attributes = {
  .name = "FaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for ActuatorTask */
osThreadId_t ActuatorTaskHandle;
const osThreadAttr_t ActuatorTask_attributes = {
  .name = "ActuatorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UartTask */
osThreadId_t UartTaskHandle;
const osThreadAttr_t UartTask_attributes = {
  .name = "UartTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for SensorQueueHandle */
osMessageQueueId_t SensorQueueHandleHandle;
const osMessageQueueAttr_t SensorQueueHandle_attributes = {
  .name = "SensorQueueHandle"
};
/* Definitions for FaultQueueHandle */
osMessageQueueId_t FaultQueueHandleHandle;
const osMessageQueueAttr_t FaultQueueHandle_attributes = {
  .name = "FaultQueueHandle"
};
/* USER CODE BEGIN PV */
/* Sensor data payload */
typedef struct {
  float distance_cm;
  uint8_t distance_ok;
  float tilt_deg;
  uint8_t tilt_ok;
  float temperature;
  float humidity;
  uint8_t dht_ok;
} SensorData_t;

/* System health classification */
typedef enum { SYS_HEALTHY, SYS_WARNING, SYS_FAULT } SystemState;

/* Processed fault state payload */
typedef struct {
  SystemState state;
  char status_msg[32];
  SensorData_t data;
} FaultResult_t;

static uint8_t mpu_ok, dht_ok;
static float baseline_tilt = 0.0f;
osMessageQueueId_t FaultQueue2Handle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
void StartDefaultTask(void *argument);
void StartSensorTask(void *argument);
void StartTask03(void *argument);
void StartActuatorTask(void *argument);
void StartUartTask(void *argument);

/* USER CODE BEGIN PFP */
static void DWT_Init(void);
static void Run_POST(void);
static void Peripherals_Init(void);
static void uart_print(const char *str);
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  /* UART boot test */
  uart_print("\r\n--- UART OK ---\r\n");

  Peripherals_Init();
  Run_POST();
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

  /* Create the queue(s) */
  /* creation of SensorQueueHandle */
  SensorQueueHandleHandle = osMessageQueueNew (16, sizeof(uint16_t), &SensorQueueHandle_attributes);

  /* creation of FaultQueueHandle */
  FaultQueueHandleHandle = osMessageQueueNew (16, sizeof(uint16_t), &FaultQueueHandle_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  osMessageQueueDelete(SensorQueueHandleHandle);
  SensorQueueHandleHandle = osMessageQueueNew(4, sizeof(SensorData_t), &SensorQueueHandle_attributes);
  osMessageQueueDelete(FaultQueueHandleHandle);
  FaultQueueHandleHandle = osMessageQueueNew(4, sizeof(FaultResult_t), &FaultQueueHandle_attributes);
  FaultQueue2Handle = osMessageQueueNew(4, sizeof(FaultResult_t), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of SensorTask */
  SensorTaskHandle = osThreadNew(StartSensorTask, NULL, &SensorTask_attributes);

  /* creation of FaultTask */
  FaultTaskHandle = osThreadNew(StartTask03, NULL, &FaultTask_attributes);

  /* creation of ActuatorTask */
  ActuatorTaskHandle = osThreadNew(StartActuatorTask, NULL, &ActuatorTask_attributes);

  /* creation of UartTask */
  UartTaskHandle = osThreadNew(StartUartTask, NULL, &UartTask_attributes);

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
  while (1) {
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  hi2c1.Init.ClockSpeed = 100000;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 1500;
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
  HAL_GPIO_WritePin(GPIOC, DHT11_DATA_Pin|HCSR04_TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DHT11_DATA_Pin HCSR04_TRIG_Pin */
  GPIO_InitStruct.Pin = DHT11_DATA_Pin|HCSR04_TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BUZZER_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* ---- Lightweight UART helpers (no printf float dependency) ---- */
static void uart_print(const char *str) {
  while (*str) {
    HAL_UART_Transmit(&huart2, (uint8_t *)str, 1, HAL_MAX_DELAY);
    str++;
  }
}

/* Print float with 1 decimal place, e.g. 23.5 */
static void uart_print_float(float val) {
  char buf[16];
  if (val < 0) {
    uart_print("-");
    val = -val;
  }
  int whole = (int)val;
  int frac = (int)((val - whole) * 10 + 0.5f);
  if (frac >= 10) {
    whole++;
    frac = 0;
  }
  int len = snprintf(buf, sizeof(buf), "%d.%d", whole, frac);
  HAL_UART_Transmit(&huart2, (uint8_t *)buf, len, HAL_MAX_DELAY);
}

/* Print unsigned integer */
static void uart_print_uint(unsigned int val) {
  char buf[12];
  int len = snprintf(buf, sizeof(buf), "%u", val);
  HAL_UART_Transmit(&huart2, (uint8_t *)buf, len, HAL_MAX_DELAY);
}

static void DWT_Init(void) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void Run_POST(void) {
  uart_print("========================================\r\n");
  uart_print(" Smart Embedded Robot Controller - POST\r\n");
  uart_print("========================================\r\n");

  mpu_ok = MPU6050_Init();
  uart_print("[POST] MPU6050 (I2C1)........... ");
  if (mpu_ok) {
      uart_print("PASS\r\n");
      /* Calibrate baseline tilt so "ideal" = 0 degrees */
      float sum_tilt = 0;
      for (int i = 0; i < 10; i++) {
          float t = 0;
          MPU6050_ReadTilt(&t);
          sum_tilt += t;
          HAL_Delay(10);
      }
      baseline_tilt = sum_tilt / 10.0f;
  } else {
      uart_print("FAIL\r\n");
  }

  HCSR04_Trigger();
  HAL_Delay(60);
  HCSR04_CheckTimeout();
  uart_print("[POST] HC-SR04 (Trig/Echo)...... ");
  uart_print(HCSR04_IsHealthy() ? "PASS\r\n" : "FAIL\r\n");

  DHT11_Data post_dht;
  dht_ok = DHT11_Read(&post_dht);
  uart_print("[POST] DHT11 (Temp/Humidity).... ");
  uart_print(dht_ok ? "PASS\r\n" : "FAIL\r\n");

  uart_print("[POST] Servo PWM (TIM3)......... INIT OK\r\n");
  uart_print("[POST] Buzzer GPIO.............. INIT OK\r\n");
  uart_print("----------------------------------------\r\n");

  if (mpu_ok && HCSR04_IsHealthy() && dht_ok)
    uart_print("System Status: ALL SENSORS OK - READY\r\n");
  else
    uart_print("System Status: WARNING - sensor(s) failed POST\r\n");
  uart_print("========================================\r\n\r\n");
}

static void Peripherals_Init(void) {
  DWT_Init();
  HCSR04_Init();
  DHT11_Init();
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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartSensorTask */
/**
* @brief Function implementing the SensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void *argument)
{
  /* USER CODE BEGIN StartSensorTask */
  SensorData_t d;
  DHT11_Data dht;

  for(;;)
  {
    memset(&d, 0, sizeof(d));

    /* Ultrasonic sensor reading */
    HCSR04_Trigger();
    osDelay(60);
    HCSR04_CheckTimeout();
    d.distance_ok = HCSR04_IsHealthy();
    d.distance_cm = HCSR04_GetDistance();

    /* MPU6050 tilt reading */
    d.tilt_ok = mpu_ok ? MPU6050_ReadTilt(&d.tilt_deg) : 0;
    if (d.tilt_ok) {
      d.tilt_deg -= baseline_tilt;
      if (d.tilt_deg < 0) d.tilt_deg = -d.tilt_deg;
    }

    /* DHT11 environment reading */
    d.dht_ok = DHT11_Read(&dht);
    if (d.dht_ok) {
      d.temperature = dht.temperature;
      d.humidity = dht.humidity;
    }

    /* Dispatch to fault evaluation task */
    osMessageQueuePut(SensorQueueHandleHandle, &d, 0, 0);
    osDelay(1000);
  }
  /* USER CODE END StartSensorTask */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the FaultTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  SensorData_t d;
  FaultResult_t f;
  uint8_t obstacle_active = 0;

  for(;;)
  {
    if (osMessageQueueGet(SensorQueueHandleHandle, &d, NULL, osWaitForever) == osOK)
    {
      f.state = SYS_HEALTHY;
      f.data = d;
      strcpy(f.status_msg, "OK");

      /* Peripheral health checks */
      if (!d.distance_ok)
        { f.state = SYS_WARNING; strcpy(f.status_msg, "HC-SR04 TIMEOUT"); }
      if (mpu_ok && !d.tilt_ok)
        { f.state = SYS_WARNING; strcpy(f.status_msg, "MPU6050 COMM FAIL"); }
      if (!d.dht_ok)
        { f.state = SYS_WARNING; strcpy(f.status_msg, "DHT11 COMM FAIL"); }
      if (d.dht_ok && d.temperature > OVERHEAT_THRESHOLD_C)
        { f.state = SYS_WARNING; strcpy(f.status_msg, "OVERHEAT"); }

      /* Obstacle proximity evaluation with hysteresis */
      if (d.distance_ok) {
        if (d.distance_cm < OBSTACLE_THRESHOLD_CM)
          obstacle_active = 1;
        else if (d.distance_cm > OBSTACLE_THRESHOLD_CM + 2.0f)
          obstacle_active = 0;

        if (obstacle_active)
          { f.state = SYS_FAULT; strcpy(f.status_msg, "OBSTACLE TOO CLOSE"); }
        else if (d.distance_cm < NEARBY_THRESHOLD_CM && f.state != SYS_FAULT)
          { f.state = SYS_WARNING; strcpy(f.status_msg, "OBJECT NEARBY"); }
      }

      /* Manual override */
      if (emergency_stop_flag)
        { f.state = SYS_FAULT; strcpy(f.status_msg, "MANUAL EMERGENCY STOP"); }

      /* Dispatch state to consumers */
      osMessageQueuePut(FaultQueueHandleHandle, &f, 0, 0);
      osMessageQueuePut(FaultQueue2Handle, &f, 0, 0);
    }
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartActuatorTask */
/**
* @brief Function implementing the ActuatorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartActuatorTask */
void StartActuatorTask(void *argument)
{
  /* USER CODE BEGIN StartActuatorTask */
  FaultResult_t current_f = {0};
  current_f.state = SYS_HEALTHY;
  current_f.data.distance_ok = 1;
  current_f.data.distance_cm = 999.0f;

  for(;;)
  {
    /* Handle manual emergency stop */
    if (emergency_stop_flag) {
      Buzzer_Off();
      osDelay(50);
      continue;
    }

    /* Update current state if new data is available */
    FaultResult_t temp_f;
    if (osMessageQueueGet(FaultQueueHandleHandle, &temp_f, NULL, 50) == osOK) {
      current_f = temp_f;
    }

    /* Evaluate proximity threshold (with basic noise rejection) */
    if (current_f.data.distance_ok && current_f.data.distance_cm < 30.0f && current_f.data.distance_cm >= 2.0f) {
      Buzzer_On();
    } else {
      Buzzer_Off();
      Servo_SweepStep();
    }
  }
  /* USER CODE END StartActuatorTask */
}

/* USER CODE BEGIN Header_StartUartTask */
/**
* @brief Function implementing the UartTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUartTask */
void StartUartTask(void *argument)
{
  /* USER CODE BEGIN StartUartTask */
  FaultResult_t f;

  for(;;)
  {
    if (osMessageQueueGet(FaultQueue2Handle, &f, NULL, osWaitForever) == osOK)
    {
      uart_print("Distance : ");
      if (f.data.distance_ok) {
        uart_print_float(f.data.distance_cm);
        uart_print(" cm\r\n");
      } else {
        uart_print("--- (no echo)\r\n");
      }

      uart_print("Tilt     : ");
      if (f.data.tilt_ok) {
        uart_print_float(f.data.tilt_deg);
        uart_print(" deg\r\n");
      } else {
        uart_print("--- (sensor unavailable)\r\n");
      }

      uart_print("Temp/RH  : ");
      if (f.data.dht_ok) {
        uart_print_float(f.data.temperature);
        uart_print(" C / ");
        uart_print_float(f.data.humidity);
        uart_print(" %\r\n");
      } else {
        uart_print("--- (sensor unavailable)\r\n");
      }

      uart_print("\r\nMotor    : ");
      uart_print(f.state == SYS_FAULT ? "STOPPED\r\n" : "SCANNING\r\n");

      uart_print("Servo    : ");
      uart_print_uint(Servo_GetAngle());
      uart_print(" deg\r\n");

      uart_print("Status   : ");
      uart_print(f.status_msg);
      uart_print("\r\n");

      uart_print("System   : ");
      if (f.state == SYS_HEALTHY) uart_print("HEALTHY\r\n");
      else if (f.state == SYS_WARNING) uart_print("WARNING\r\n");
      else uart_print("FAULT\r\n");
      uart_print("----------------------------------------\r\n");
    }
  }
  /* USER CODE END StartUartTask */
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
  while (1) {
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
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
