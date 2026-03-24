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
#include "line_sensor.h"
#include "uart_log.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define APP_HEARTBEAT_LOG_MS             2000U
#define APP_LINE_POLL_MS                 50U
#define APP_I2C_SCAN_START_ADDR          0x08U
#define APP_I2C_SCAN_END_ADDR            0x77U
#define APP_LINE_BLACK_ACTIVE_LOW        1U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint32_t heartbeat_tick = 0U;
static LineSensor_t line_sensor;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void App_ApplyLineToLed(uint8_t has_line);
static void App_DetectLineSensorAddress(void);
static void App_LogLineSnapshot(const char *tag, const LineSensorSnapshot_t *snapshot);

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

static void App_ApplyLineToLed(uint8_t has_line)
{
  HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, (has_line != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void App_DetectLineSensorAddress(void)
{
  uint8_t found_addr = 0U;
  HAL_StatusTypeDef ping_status = LineSensor_Ping(&line_sensor, 3U);

  if (ping_status == HAL_OK)
  {
    UartLog_PrintTimestamped(
      "LINE",
      "I2C device ACK at configured addr=0x%02X\r\n",
      (unsigned int)LineSensor_GetAddress7bit(&line_sensor));
    return;
  }

  UartLog_PrintTimestamped(
    "LINE",
    "No ACK at addr=0x%02X, scanning I2C bus...\r\n",
    (unsigned int)LineSensor_GetAddress7bit(&line_sensor));

  if (LineSensor_ScanFirst(
        &line_sensor,
        (uint8_t)APP_I2C_SCAN_START_ADDR,
        (uint8_t)APP_I2C_SCAN_END_ADDR,
        &found_addr) != 0U)
  {
    UartLog_PrintTimestamped(
      "LINE",
      "Detected I2C device at addr=0x%02X (updated)\r\n",
      (unsigned int)found_addr);
  }
  else
  {
    UartLog_PrintTimestamped(
      "LINE",
      "No I2C device found in range 0x%02X..0x%02X (check wiring, power, pull-up)\r\n",
      (unsigned int)APP_I2C_SCAN_START_ADDR,
      (unsigned int)APP_I2C_SCAN_END_ADDR);
  }
}

static void App_LogLineSnapshot(const char *tag, const LineSensorSnapshot_t *snapshot)
{
  uint8_t raw_nibble;

  if (snapshot == NULL)
  {
    return;
  }

  raw_nibble = (uint8_t)(snapshot->raw_data & 0x0FU);

  UartLog_PrintTimestamped(
    tag,
    "raw=0x%02X raw4=0x%X sensor=0x%X line=%u,%u,%u,%u has_line=%u status=%s ev=0x%02X\r\n",
    (unsigned int)snapshot->raw_data,
    (unsigned int)raw_nibble,
    (unsigned int)snapshot->sensor_nibble,
    (unsigned int)snapshot->line3,
    (unsigned int)snapshot->line2,
    (unsigned int)snapshot->line1,
    (unsigned int)snapshot->line0,
    (unsigned int)snapshot->has_line,
    LineSensor_StatusName(snapshot->status),
    (unsigned int)snapshot->event_flags);
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

  LineSensor_Init(&line_sensor, &hi2c1, LINE_SENSOR_DEFAULT_ADDR_7BIT);
#if (APP_LINE_BLACK_ACTIVE_LOW != 0U)
  LineSensor_SetPolarity(&line_sensor, LINE_SENSOR_POLARITY_ACTIVE_LOW);
#else
  LineSensor_SetPolarity(&line_sensor, LINE_SENSOR_POLARITY_ACTIVE_HIGH);
#endif
  LineSensor_SetTimeout(&line_sensor, LINE_SENSOR_DEFAULT_TIMEOUT_MS);
  LineSensor_SetPollInterval(&line_sensor, APP_LINE_POLL_MS);
  App_DetectLineSensorAddress();
  heartbeat_tick = HAL_GetTick();
  LineSensor_ResetScheduler(&line_sensor, heartbeat_tick);
  App_ApplyLineToLed(0U);
  UartLog_PrintTimestamped(
    "INIT",
    "Line sensor ready (addr=0x%02X, poll=%lu ms, polarity=%s)\r\n",
    (unsigned int)LineSensor_GetAddress7bit(&line_sensor),
    (uint32_t)APP_LINE_POLL_MS,
    LineSensor_PolarityName(LineSensor_GetPolarity(&line_sensor)));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();
    const LineSensorSnapshot_t *snapshot;

    if (LineSensor_Update(&line_sensor, now) != 0U)
    {
      snapshot = LineSensor_GetSnapshot(&line_sensor);

      if (snapshot != NULL)
      {
        if (snapshot->status == HAL_OK)
        {
          App_ApplyLineToLed(snapshot->has_line);
        }
        else
        {
          App_ApplyLineToLed(0U);
        }

        if ((snapshot->event_flags & (LINE_SENSOR_EVENT_STATUS_CHANGED |
                                      LINE_SENSOR_EVENT_STATE_CHANGED |
                                      LINE_SENSOR_EVENT_LINE_ENTER |
                                      LINE_SENSOR_EVENT_LINE_EXIT)) != 0U)
        {
          App_LogLineSnapshot("LINE", snapshot);
        }
      }
    }

    if ((now - heartbeat_tick) >= APP_HEARTBEAT_LOG_MS)
    {
      heartbeat_tick = now;
      snapshot = LineSensor_GetSnapshot(&line_sensor);
      App_LogLineSnapshot("HB", snapshot);
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
  RCC_OscInitStruct.PLL.PLLN = 101;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
