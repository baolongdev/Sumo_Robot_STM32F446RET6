/*
 * Basic line sensor polling sample
 * Poll by scheduler, print L3,L2,L1,L0, drive LED by has_line.
 * Copy into Core/Src/main.c USER CODE regions.
 */

/* USER CODE BEGIN Includes */
#include "modules/sensors/line_sensor.h"
#include "modules/logging/uart_log.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
#define APP_LINE_POLL_MS      50U
/* USER CODE END PD */

/* USER CODE BEGIN PV */
static LineSensor_t line_sensor;
/* USER CODE END PV */

/* USER CODE BEGIN 0 */
static void App_ApplyLineToLed(uint8_t has_line)
{
  HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, (has_line != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
/* USER CODE END 0 */

/* USER CODE BEGIN 2 */
UartLog_Init(&huart1);
UartLog_SetEnabled(1U);
LineSensor_Init(&line_sensor, &hi2c1, LINE_SENSOR_DEFAULT_ADDR_7BIT);
LineSensor_SetPolarity(&line_sensor, LINE_SENSOR_POLARITY_ACTIVE_LOW);
LineSensor_SetPollInterval(&line_sensor, APP_LINE_POLL_MS);
LineSensor_ResetScheduler(&line_sensor, HAL_GetTick());
/* USER CODE END 2 */

/* USER CODE BEGIN 3 */
{
  uint32_t now = HAL_GetTick();
  const LineSensorSnapshot_t *snapshot;

  if (LineSensor_Update(&line_sensor, now) != 0U)
  {
    snapshot = LineSensor_GetSnapshot(&line_sensor);
    if ((snapshot != NULL) && (snapshot->status == HAL_OK))
    {
      App_ApplyLineToLed(snapshot->has_line);
      UartLog_PrintTimestamped(
        "LINE",
        "raw4=0x%X sensor=0x%X line=%u,%u,%u,%u has_line=%u\r\n",
        (unsigned int)(snapshot->raw_data & 0x0FU),
        (unsigned int)snapshot->sensor_nibble,
        (unsigned int)snapshot->line3,
        (unsigned int)snapshot->line2,
        (unsigned int)snapshot->line1,
        (unsigned int)snapshot->line0,
        (unsigned int)snapshot->has_line);
    }
  }
}
/* USER CODE END 3 */


