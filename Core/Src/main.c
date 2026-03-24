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
#include "led_manager.h"
#include "uart_log.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define APP_HEARTBEAT_LOG_MS             2000U
#define APP_LED_TEST_FAST_MS             3500U
#define APP_LED_TEST_SLOW_MS             3500U
#define APP_LED_TEST_LONG_MS             5000U
#define APP_LED_TEST_ON_HOLD_MS          1800U
#define APP_LED_TEST_OFF_HOLD_MS         1800U
#define APP_LED_CUSTOM_LONG_BLINK_MS     1200U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint32_t heartbeat_tick = 0U;
static uint32_t led_test_tick = 0U;
static uint8_t led_test_step = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void App_LedTestEnterStep(uint8_t step);

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

static void App_LedTestEnterStep(uint8_t step)
{
  led_test_step = step;
  led_test_tick = HAL_GetTick();

  switch (step)
  {
    case 0U:
      LedManager_SetMode(LED_MODE_BLINK_FAST);
      UartLog_PrintTimestamped("LED", "TEST step=FAST_BLINK\r\n");
      break;
    case 1U:
      LedManager_SetMode(LED_MODE_BLINK_SLOW);
      UartLog_PrintTimestamped("LED", "TEST step=SLOW_BLINK\r\n");
      break;
    case 2U:
      LedManager_SetCustomBlink(APP_LED_CUSTOM_LONG_BLINK_MS);
      LedManager_SetMode(LED_MODE_BLINK_CUSTOM);
      UartLog_PrintTimestamped("LED", "TEST step=LONG_BLINK(%lu ms)\r\n", (uint32_t)APP_LED_CUSTOM_LONG_BLINK_MS);
      break;
    case 3U:
      LedManager_SetMode(LED_MODE_ON);
      UartLog_PrintTimestamped("LED", "TEST step=ON_HOLD\r\n");
      break;
    default:
      LedManager_SetMode(LED_MODE_OFF);
      UartLog_PrintTimestamped("LED", "TEST step=OFF_HOLD\r\n");
      break;
  }
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

  LedManager_Init();
  LedManager_SetCustomBlink(APP_LED_CUSTOM_LONG_BLINK_MS);
  heartbeat_tick = HAL_GetTick();
  App_LedTestEnterStep(0U);
  UartLog_PrintTimestamped("INIT", "LED test sequencer started\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();
    LedManager_Update();

    if (led_test_step == 0U)
    {
      if ((now - led_test_tick) >= APP_LED_TEST_FAST_MS)
      {
        App_LedTestEnterStep(1U);
      }
    }
    else if (led_test_step == 1U)
    {
      if ((now - led_test_tick) >= APP_LED_TEST_SLOW_MS)
      {
        App_LedTestEnterStep(2U);
      }
    }
    else if (led_test_step == 2U)
    {
      if ((now - led_test_tick) >= APP_LED_TEST_LONG_MS)
      {
        App_LedTestEnterStep(3U);
      }
    }
    else if (led_test_step == 3U)
    {
      if ((now - led_test_tick) >= APP_LED_TEST_ON_HOLD_MS)
      {
        App_LedTestEnterStep(4U);
      }
    }
    else
    {
      if ((now - led_test_tick) >= APP_LED_TEST_OFF_HOLD_MS)
      {
        App_LedTestEnterStep(0U);
      }
    }

    if ((now - heartbeat_tick) >= APP_HEARTBEAT_LOG_MS)
    {
      heartbeat_tick = now;
      UartLog_PrintTimestamped(
        "HB",
        "uptime=%lu ms, LED=%s, step=%u\r\n",
        now,
        LedManager_ModeName(LedManager_GetMode()),
        led_test_step);
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
