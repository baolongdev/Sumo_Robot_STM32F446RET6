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
#include "motor.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TEST_PROFILE_TARGET_PERCENT      55U
#define TEST_PROFILE_RAMP_MS             1000U
#define TEST_PROFILE_RUN_MS              1400U
#define TEST_PROFILE_BRAKE_MS            220U
#define TEST_PIVOT_PERCENT               40U
#define TEST_PIVOT_MS                    650U
#define TEST_RETREAT_PERCENT             50U
#define TEST_RETREAT_MS                  800U
#define TEST_PAUSE_MS                    700U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static MotorController_t motor_controller;
static uint8_t test_phase = 0U;
static uint32_t phase_tick = 0U;
static uint32_t led_tick = 0U;
static uint8_t estop_latched = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint8_t App_IsBtnB0PressedEdge(void)
{
  static uint8_t last_raw = 1U;
  static uint8_t stable = 1U;
  static uint32_t stable_tick = 0U;
  uint8_t raw = (HAL_GPIO_ReadPin(BTN_B0_GPIO_Port, BTN_B0_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
  uint8_t edge = 0U;
  uint32_t now = HAL_GetTick();

  if (raw != last_raw)
  {
    last_raw = raw;
    stable_tick = now;
  }

  if ((now - stable_tick) >= 30U)
  {
    if (stable != raw)
    {
      stable = raw;
      if (stable != 0U)
      {
        edge = 1U;
      }
    }
  }

  return edge;
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
  MotorController_InitDefault(&motor_controller);
  /* Set 1 if any wheel direction is physically reversed on your chassis. */
  MotorController_SetInversion(&motor_controller, 0U, 0U);
  MotorController_SetStopMode(&motor_controller, MOTOR_STOP_BRAKE);
  MotorController_SetDriveMode(&motor_controller, MOTOR_DRIVE_MODE_ATTACK);
  MotorController_SetCommandTimeout(&motor_controller, 0U); /* disable watchdog while bench testing */
  MotorController_SetDirectionSafety(&motor_controller, 80U, 40U);
  MotorController_SetMinDrivePercent(&motor_controller, 18U, 10U);
  if (MotorController_Start(&motor_controller) != HAL_OK)
  {
    Error_Handler();
  }
  MotorController_Stop(&motor_controller);
  test_phase = 0U;
  phase_tick = HAL_GetTick();
  led_tick = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (App_IsBtnB0PressedEdge() != 0U)
    {
      if (estop_latched == 0U)
      {
        MotorController_EmergencyStop(&motor_controller);
        estop_latched = 1U;
      }
      else
      {
        MotorController_EmergencyRelease(&motor_controller);
        MotorController_ClearFaults(&motor_controller, MOTOR_FAULT_ESTOP);
        MotorController_Stop(&motor_controller);
        estop_latched = 0U;
        test_phase = 0U;
        phase_tick = HAL_GetTick();
      }
    }

    MotorController_Update(&motor_controller);

    if (MotorController_IsEmergencyStopped(&motor_controller) != 0U)
    {
      if ((HAL_GetTick() - led_tick) >= 120U)
      {
        HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
        led_tick = HAL_GetTick();
      }
      continue;
    }

    if (MotorController_GetFaults(&motor_controller) != MOTOR_FAULT_NONE)
    {
      HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_SET);
      continue;
    }
    else
    {
      if ((HAL_GetTick() - led_tick) >= 500U)
      {
        HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
        led_tick = HAL_GetTick();
      }
    }

    switch (test_phase)
    {
      case 0:
        MotorController_ProfileStart(
          &motor_controller,
          TEST_PROFILE_TARGET_PERCENT,
          TEST_PROFILE_RAMP_MS,
          TEST_PROFILE_RUN_MS,
          TEST_PROFILE_BRAKE_MS);
        test_phase = 1U;
        break;

      case 1:
        if (MotorController_ProfileIsBusy(&motor_controller) == 0U)
        {
          MotorController_PivotLeft(&motor_controller, TEST_PIVOT_PERCENT);
          phase_tick = HAL_GetTick();
          test_phase = 2U;
        }
        break;

      case 2:
        if ((HAL_GetTick() - phase_tick) >= TEST_PIVOT_MS)
        {
          MotorController_PivotRight(&motor_controller, TEST_PIVOT_PERCENT);
          phase_tick = HAL_GetTick();
          test_phase = 3U;
        }
        break;

      case 3:
        if ((HAL_GetTick() - phase_tick) >= TEST_PIVOT_MS)
        {
          MotorController_Retreat(&motor_controller, TEST_RETREAT_PERCENT);
          phase_tick = HAL_GetTick();
          test_phase = 4U;
        }
        break;

      case 4:
        if ((HAL_GetTick() - phase_tick) >= TEST_RETREAT_MS)
        {
          MotorController_Stop(&motor_controller);
          phase_tick = HAL_GetTick();
          test_phase = 5U;
        }
        break;

      default:
        if ((HAL_GetTick() - phase_tick) >= TEST_PAUSE_MS)
        {
          test_phase = 0U;
        }
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
