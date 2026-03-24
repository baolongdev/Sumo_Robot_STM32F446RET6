/*
 * Sumo search/attack behavior sample (non-blocking)
 * Copy into Core/Src/main.c USER CODE regions.
 */

/* USER CODE BEGIN Includes */
#include "motor.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
static MotorController_t motor_controller;
/* Replace these flags with your real sensor outputs. */
static uint8_t line_left_detected = 0U;
static uint8_t line_right_detected = 0U;
static uint8_t enemy_detected = 0U;
/* USER CODE END PV */

/* USER CODE BEGIN 2 */
MotorController_InitDefault(&motor_controller);
MotorController_SetInversion(&motor_controller, 0U, 0U);
MotorController_SetStopMode(&motor_controller, MOTOR_STOP_BRAKE);
MotorController_SetDirectionSafety(&motor_controller, 80U, 40U);
MotorController_SetMinDrivePercent(&motor_controller, 18U, 10U);
MotorController_SetCommandTimeout(&motor_controller, 500U);
if (MotorController_Start(&motor_controller) != HAL_OK)
{
  Error_Handler();
}
MotorController_SetDriveMode(&motor_controller, MOTOR_DRIVE_MODE_SEARCH);
/* USER CODE END 2 */

/* USER CODE BEGIN 3 */
MotorController_Update(&motor_controller);

/* 1) Boundary first: never cross edge line. */
if ((line_left_detected != 0U) || (line_right_detected != 0U))
{
  MotorController_SetDriveMode(&motor_controller, MOTOR_DRIVE_MODE_ATTACK);
  MotorController_Retreat(&motor_controller, 55U);
}
/* 2) Attack when enemy is detected. */
else if (enemy_detected != 0U)
{
  MotorController_SetDriveMode(&motor_controller, MOTOR_DRIVE_MODE_ATTACK);
  MotorController_Charge(&motor_controller, 80U);
}
/* 3) Search when no enemy. */
else
{
  MotorController_SetDriveMode(&motor_controller, MOTOR_DRIVE_MODE_SEARCH);
  MotorController_PivotLeft(&motor_controller, 28U);
}
/* USER CODE END 3 */
