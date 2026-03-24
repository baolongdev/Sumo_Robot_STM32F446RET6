/*
 * Sumo boundary guard sample (line sensor + motor)
 * Priority:
 *   1) If edge line detected -> retreat immediately
 *   2) Else run normal attack/search logic
 * Copy into Core/Src/main.c USER CODE regions.
 */

/* USER CODE BEGIN Includes */
#include "line_sensor.h"
#include "motor.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
#define APP_LINE_POLL_MS      40U
/* USER CODE END PD */

/* USER CODE BEGIN PV */
static LineSensor_t line_sensor;
static MotorController_t motor_controller;
/* Mock flags - replace with your real enemy detection pipeline. */
static uint8_t enemy_detected = 0U;
/* USER CODE END PV */

/* USER CODE BEGIN 2 */
LineSensor_Init(&line_sensor, &hi2c1, LINE_SENSOR_DEFAULT_ADDR_7BIT);
LineSensor_SetPolarity(&line_sensor, LINE_SENSOR_POLARITY_ACTIVE_LOW);
LineSensor_SetPollInterval(&line_sensor, APP_LINE_POLL_MS);
LineSensor_ResetScheduler(&line_sensor, HAL_GetTick());

MotorController_InitDefault(&motor_controller);
MotorController_SetInversion(&motor_controller, 0U, 0U);
MotorController_SetStopMode(&motor_controller, MOTOR_STOP_BRAKE);
if (MotorController_Start(&motor_controller) != HAL_OK)
{
  Error_Handler();
}
/* USER CODE END 2 */

/* USER CODE BEGIN 3 */
{
  uint32_t now = HAL_GetTick();
  const LineSensorSnapshot_t *snapshot = NULL;

  MotorController_Update(&motor_controller);
  (void)LineSensor_Update(&line_sensor, now);
  snapshot = LineSensor_GetSnapshot(&line_sensor);

  if ((snapshot != NULL) && (snapshot->status == HAL_OK) && (snapshot->has_line != 0U))
  {
    /* Edge safety first: pull back from border. */
    MotorController_Retreat(&motor_controller, 60U);
  }
  else if (enemy_detected != 0U)
  {
    MotorController_Charge(&motor_controller, 80U);
  }
  else
  {
    MotorController_PivotLeft(&motor_controller, 25U);
  }
}
/* USER CODE END 3 */

