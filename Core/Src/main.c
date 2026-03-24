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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "ultrasonic_manager.h"
#include "uart_log.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define APP_HEARTBEAT_LOG_MS             2000U
#define APP_US_LOG_MS                    120U
#define APP_US_DEBUG_STATUS              1U

#define APP_US_ECHO_RISE_TIMEOUT_US      3000U
#define APP_US_ECHO_FALL_TIMEOUT_US      25000U
#define APP_US_INTER_SENSOR_GAP_US       3000U
#define APP_US_KALMAN_Q                  12.0f
#define APP_US_KALMAN_R                  16.0f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint32_t heartbeat_tick = 0U;
static uint32_t us_log_tick = 0U;
static UltrasonicManager_t ultrasonic_manager;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void App_LogUltrasonicSummary(const char *tag);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void App_UartRawPrint(const char *text)
{
  uint16_t len;

  if (text == NULL)
  {
    return;
  }

  len = (uint16_t)strlen(text);
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)text, len, 100U);
}

static void App_LogUltrasonicSummary(const char *tag)
{
  const UltrasonicReading_t *left = UltrasonicManager_GetReading(&ultrasonic_manager, ULTRASONIC_SENSOR_LEFT);
  const UltrasonicReading_t *mid = UltrasonicManager_GetReading(&ultrasonic_manager, ULTRASONIC_SENSOR_MID);
  const UltrasonicReading_t *right = UltrasonicManager_GetReading(&ultrasonic_manager, ULTRASONIC_SENSOR_RIGHT);
  uint8_t echo_l = (HAL_GPIO_ReadPin(US_LEFT_ECHO_GPIO_Port, US_LEFT_ECHO_Pin) == GPIO_PIN_SET) ? 1U : 0U;
  uint8_t echo_m = (HAL_GPIO_ReadPin(US_MID_ECHO_GPIO_Port, US_MID_ECHO_Pin) == GPIO_PIN_SET) ? 1U : 0U;
  uint8_t echo_r = (HAL_GPIO_ReadPin(US_RIGHT_ECHO_GPIO_Port, US_RIGHT_ECHO_Pin) == GPIO_PIN_SET) ? 1U : 0U;

  if ((left == NULL) || (mid == NULL) || (right == NULL))
  {
    return;
  }

#if (APP_US_DEBUG_STATUS != 0U)
  UartLog_PrintTimestamped(
    tag,
    "US cm L=%u(raw=%u,%s) M=%u(raw=%u,%s) R=%u(raw=%u,%s) echo=%u,%u,%u\r\n",
    (unsigned int)left->distance_cm,
    (unsigned int)left->raw_distance_cm,
    UltrasonicManager_StatusName(left->status),
    (unsigned int)mid->distance_cm,
    (unsigned int)mid->raw_distance_cm,
    UltrasonicManager_StatusName(mid->status),
    (unsigned int)right->distance_cm,
    (unsigned int)right->raw_distance_cm,
    UltrasonicManager_StatusName(right->status),
    (unsigned int)echo_l,
    (unsigned int)echo_m,
    (unsigned int)echo_r);
#else
  UartLog_PrintTimestamped(
    tag,
    "US cm L=%u M=%u R=%u\r\n",
    (unsigned int)left->distance_cm,
    (unsigned int)mid->distance_cm,
    (unsigned int)right->distance_cm);
#endif
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
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  UartLog_Init(&huart1);
  UartLog_SetEnabled(1U);
  App_UartRawPrint("\r\n[RAW] UART1 ready @115200\r\n");
  UartLog_Print("\r\n\r\n=== SUMO ROBOT APP BOOT ===\r\n");

  heartbeat_tick = HAL_GetTick();
  us_log_tick = heartbeat_tick;

  UltrasonicManager_InitDefault(&ultrasonic_manager, &htim1);
  ultrasonic_manager.config.echo_rise_timeout_us = APP_US_ECHO_RISE_TIMEOUT_US;
  ultrasonic_manager.config.echo_fall_timeout_us = APP_US_ECHO_FALL_TIMEOUT_US;
  ultrasonic_manager.config.inter_sensor_gap_us = APP_US_INTER_SENSOR_GAP_US;
  UltrasonicManager_SetKalman(&ultrasonic_manager, 1U, APP_US_KALMAN_Q, APP_US_KALMAN_R);
  if (UltrasonicManager_Start(&ultrasonic_manager) != HAL_OK)
  {
    Error_Handler();
  }
  UartLog_PrintTimestamped(
    "INIT",
    "Ultrasonic read ready (LEFT/MID/RIGHT, fast profile: gap=%uus, kalman=ON)\r\n",
    (unsigned int)APP_US_INTER_SENSOR_GAP_US);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();
    (void)UltrasonicManager_Update(&ultrasonic_manager, now);

    if ((now - us_log_tick) >= APP_US_LOG_MS)
    {
      us_log_tick = now;
      App_LogUltrasonicSummary("US");
    }

    if ((now - heartbeat_tick) >= APP_HEARTBEAT_LOG_MS)
    {
      heartbeat_tick = now;
      App_LogUltrasonicSummary("HB");
      App_UartRawPrint("[RAW] heartbeat\r\n");
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
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
