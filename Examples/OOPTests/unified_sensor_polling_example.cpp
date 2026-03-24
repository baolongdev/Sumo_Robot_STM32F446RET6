/*
 * unified_sensor_polling_example.cpp
 * Class-based sample: SensorManager + unified snapshot logging
 */

#include "modules/sensors/sensor_manager.hpp"
#include "modules/logging/uart_logger.hpp"

static SensorManagerConfig g_cfg = {
  { &hi2c1, LINE_SENSOR_DEFAULT_ADDR_7BIT, 100U, 50U, 1U },
  { &htim1, 10U, 3000U, 25000U, 3000U, 1U, 12.0f, 16.0f }
};

static SensorManager g_sensor(g_cfg);

void Example_Init(void)
{
  UartLogger::instance().init(&huart1);
  UartLogger::instance().setLevel(LogLevel::Info);

  g_sensor.init();
  if (!g_sensor.start())
  {
    LOGE("EX", "sensor start failed");
  }
}

void Example_Loop(void)
{
  uint32_t now = HAL_GetTick();
  SensorEventFlags ev = g_sensor.update(now);
  const SensorManagerSnapshot &s = g_sensor.snapshot();

  if ((ev & SensorManager::EVENT_LINE_UPDATED) != 0U)
  {
    LOGI("LINE", "line=%u,%u,%u,%u has=%u",
         (unsigned int)s.line.line3,
         (unsigned int)s.line.line2,
         (unsigned int)s.line.line1,
         (unsigned int)s.line.line0,
         (unsigned int)s.line.has_line);
  }

  if ((ev & SensorManager::EVENT_ULTRASONIC_UPDATED) != 0U)
  {
    LOGI("US", "L=%u M=%u R=%u cm",
         (unsigned int)s.ultrasonic.left.distance_cm,
         (unsigned int)s.ultrasonic.mid.distance_cm,
         (unsigned int)s.ultrasonic.right.distance_cm);
  }

  UartLogger::instance().process();
}

