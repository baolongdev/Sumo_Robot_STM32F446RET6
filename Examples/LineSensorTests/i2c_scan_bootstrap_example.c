/*
 * I2C bootstrap sample for line sensor
 * 1) Ping configured address
 * 2) Scan I2C bus if no ACK
 * 3) Continue with updated address
 * Copy into Core/Src/main.c USER CODE regions.
 */

/* USER CODE BEGIN Includes */
#include "line_sensor.h"
#include "uart_log.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
#define APP_I2C_SCAN_START_ADDR    0x08U
#define APP_I2C_SCAN_END_ADDR      0x77U
/* USER CODE END PD */

/* USER CODE BEGIN PV */
static LineSensor_t line_sensor;
/* USER CODE END PV */

/* USER CODE BEGIN 0 */
static void App_DetectLineSensorAddress(void)
{
  uint8_t found_addr = 0U;

  if (LineSensor_Ping(&line_sensor, 3U) == HAL_OK)
  {
    UartLog_PrintTimestamped(
      "LINE",
      "ACK at configured addr=0x%02X\r\n",
      (unsigned int)LineSensor_GetAddress7bit(&line_sensor));
    return;
  }

  UartLog_PrintTimestamped(
    "LINE",
    "No ACK at 0x%02X, scan bus...\r\n",
    (unsigned int)LineSensor_GetAddress7bit(&line_sensor));

  if (LineSensor_ScanFirst(
        &line_sensor,
        (uint8_t)APP_I2C_SCAN_START_ADDR,
        (uint8_t)APP_I2C_SCAN_END_ADDR,
        &found_addr) != 0U)
  {
    UartLog_PrintTimestamped("LINE", "Found device at 0x%02X\r\n", (unsigned int)found_addr);
  }
  else
  {
    UartLog_PrintTimestamped("LINE", "No device in 0x%02X..0x%02X\r\n",
                             (unsigned int)APP_I2C_SCAN_START_ADDR,
                             (unsigned int)APP_I2C_SCAN_END_ADDR);
  }
}
/* USER CODE END 0 */

/* USER CODE BEGIN 2 */
UartLog_Init(&huart1);
UartLog_SetEnabled(1U);
LineSensor_Init(&line_sensor, &hi2c1, LINE_SENSOR_DEFAULT_ADDR_7BIT);
LineSensor_SetTimeout(&line_sensor, 100U);
App_DetectLineSensorAddress();
/* USER CODE END 2 */

