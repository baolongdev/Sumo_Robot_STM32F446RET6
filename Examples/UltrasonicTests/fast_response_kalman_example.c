/*
 * Fast-response ultrasonic sample
 * Non-blocking read with aggressive timing + fast Kalman tuning.
 * Copy into Core/Src/main.c USER CODE regions.
 */

/* USER CODE BEGIN Includes */
#include "ultrasonic_manager.h"
#include "uart_log.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
#define APP_US_LOG_MS                 120U
#define APP_US_HEARTBEAT_MS           2000U

#define APP_US_ECHO_RISE_TIMEOUT_US   3000U
#define APP_US_ECHO_FALL_TIMEOUT_US   25000U
#define APP_US_INTER_SENSOR_GAP_US    3000U

#define APP_US_KALMAN_ENABLE          1U
#define APP_US_KALMAN_Q               12.0f
#define APP_US_KALMAN_R               16.0f
/* USER CODE END PD */

/* USER CODE BEGIN PV */
static UltrasonicManager_t us_manager;
static uint32_t us_log_tick = 0U;
static uint32_t us_hb_tick = 0U;
/* USER CODE END PV */

/* USER CODE BEGIN 0 */
static void App_LogUsFast(const char *tag)
{
  const UltrasonicReading_t *left = UltrasonicManager_GetReading(&us_manager, ULTRASONIC_SENSOR_LEFT);
  const UltrasonicReading_t *mid = UltrasonicManager_GetReading(&us_manager, ULTRASONIC_SENSOR_MID);
  const UltrasonicReading_t *right = UltrasonicManager_GetReading(&us_manager, ULTRASONIC_SENSOR_RIGHT);

  if ((left == NULL) || (mid == NULL) || (right == NULL))
  {
    return;
  }

  UartLog_PrintTimestamped(
    tag,
    "US L=%u(raw=%u,%s) M=%u(raw=%u,%s) R=%u(raw=%u,%s)\\r\\n",
    (unsigned int)left->distance_cm,
    (unsigned int)left->raw_distance_cm,
    UltrasonicManager_StatusName(left->status),
    (unsigned int)mid->distance_cm,
    (unsigned int)mid->raw_distance_cm,
    UltrasonicManager_StatusName(mid->status),
    (unsigned int)right->distance_cm,
    (unsigned int)right->raw_distance_cm,
    UltrasonicManager_StatusName(right->status));
}
/* USER CODE END 0 */

/* USER CODE BEGIN 2 */
UartLog_Init(&huart1);
UartLog_SetEnabled(1U);

us_log_tick = HAL_GetTick();
us_hb_tick = us_log_tick;

UltrasonicManager_InitDefault(&us_manager, &htim1);
us_manager.config.echo_rise_timeout_us = APP_US_ECHO_RISE_TIMEOUT_US;
us_manager.config.echo_fall_timeout_us = APP_US_ECHO_FALL_TIMEOUT_US;
us_manager.config.inter_sensor_gap_us = APP_US_INTER_SENSOR_GAP_US;
UltrasonicManager_SetKalman(&us_manager, APP_US_KALMAN_ENABLE, APP_US_KALMAN_Q, APP_US_KALMAN_R);

if (UltrasonicManager_Start(&us_manager) != HAL_OK)
{
  Error_Handler();
}

UartLog_PrintTimestamped("INIT", "US fast profile ready\\r\\n");
/* USER CODE END 2 */

/* USER CODE BEGIN 3 */
{
  uint32_t now = HAL_GetTick();
  (void)UltrasonicManager_Update(&us_manager, now);

  if ((now - us_log_tick) >= APP_US_LOG_MS)
  {
    us_log_tick = now;
    App_LogUsFast("US");
  }

  if ((now - us_hb_tick) >= APP_US_HEARTBEAT_MS)
  {
    us_hb_tick = now;
    App_LogUsFast("HB");
  }
}
/* USER CODE END 3 */
