/*
 * Event-driven line sensor sample
 * Only react/log on meaningful events to reduce UART traffic.
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
    if (snapshot == NULL)
    {
      return;
    }

    if ((snapshot->event_flags & LINE_SENSOR_EVENT_STATUS_CHANGED) != 0U)
    {
      UartLog_PrintTimestamped("LINE", "status=%s\r\n", LineSensor_StatusName(snapshot->status));
    }

    if ((snapshot->event_flags & LINE_SENSOR_EVENT_LINE_ENTER) != 0U)
    {
      UartLog_PrintTimestamped(
        "LINE",
        "ENTER edge line=%u,%u,%u,%u sensor=0x%X\r\n",
        (unsigned int)snapshot->line3,
        (unsigned int)snapshot->line2,
        (unsigned int)snapshot->line1,
        (unsigned int)snapshot->line0,
        (unsigned int)snapshot->sensor_nibble);
    }

    if ((snapshot->event_flags & LINE_SENSOR_EVENT_LINE_EXIT) != 0U)
    {
      UartLog_PrintTimestamped("LINE", "EXIT edge\r\n");
    }

    if ((snapshot->event_flags & LINE_SENSOR_EVENT_STATE_CHANGED) != 0U)
    {
      UartLog_PrintTimestamped(
        "LINE",
        "state line=%u,%u,%u,%u raw4=0x%X sensor=0x%X\r\n",
        (unsigned int)snapshot->line3,
        (unsigned int)snapshot->line2,
        (unsigned int)snapshot->line1,
        (unsigned int)snapshot->line0,
        (unsigned int)(snapshot->raw_data & 0x0FU),
        (unsigned int)snapshot->sensor_nibble);
    }
  }
}
/* USER CODE END 3 */


