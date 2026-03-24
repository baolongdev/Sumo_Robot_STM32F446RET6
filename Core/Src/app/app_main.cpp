#include "app/app_main.h"

#include "gpio.h"
#include "i2c.h"
#include "modules/control/led_manager.h"
#include "modules/control/motor.h"
#include "modules/sensors/sensor_manager.hpp"
#include "tim.h"
#include "modules/logging/uart_logger.hpp"
#include "usart.h"

namespace
{
constexpr uint32_t APP_HEARTBEAT_MS = 2000U;
constexpr uint32_t APP_LINE_LOG_MS = 100U;
constexpr uint32_t APP_US_LOG_MS = 120U;
constexpr uint32_t APP_DROP_REPORT_MS = 2000U;

constexpr uint32_t KEY_LINE = 0x1001U;
constexpr uint32_t KEY_US = 0x1002U;

static SensorManagerConfig g_sensor_cfg = {
  {
    &hi2c1,
    LINE_SENSOR_DEFAULT_ADDR_7BIT,
    LINE_SENSOR_DEFAULT_TIMEOUT_MS,
    LINE_SENSOR_DEFAULT_POLL_MS,
    1U
  },
  {
    &htim1,
    10U,
    3000U,
    25000U,
    3000U,
    1U,
    12.0f,
    16.0f
  }
};

static SensorManager g_sensor_manager(g_sensor_cfg);
static MotorController_t g_motor_controller;

static uint32_t g_heartbeat_tick = 0U;
static uint32_t g_drop_tick = 0U;
static uint32_t g_last_dropped = 0U;

static void App_LogLineRateLimited(const LineSensorSnapshot &line_snapshot)
{
  UartLogger::instance().logRateLimited(
    LogLevel::Info,
    "LINE",
    KEY_LINE,
    APP_LINE_LOG_MS,
    "raw=0x%02X sensor=0x%X line=%u,%u,%u,%u has_line=%u st=%s",
    (unsigned int)line_snapshot.raw_data,
    (unsigned int)line_snapshot.sensor_nibble,
    (unsigned int)line_snapshot.line3,
    (unsigned int)line_snapshot.line2,
    (unsigned int)line_snapshot.line1,
    (unsigned int)line_snapshot.line0,
    (unsigned int)line_snapshot.has_line,
    SensorStatusName(line_snapshot.status));
}

static void App_LogUltrasonicRateLimited(const UltrasonicArraySnapshot &us_snapshot)
{
  UartLogger::instance().logRateLimited(
    LogLevel::Info,
    "US",
    KEY_US,
    APP_US_LOG_MS,
    "L=%u(raw=%u,%s) M=%u(raw=%u,%s) R=%u(raw=%u,%s)",
    (unsigned int)us_snapshot.left.distance_cm,
    (unsigned int)us_snapshot.left.raw_distance_cm,
    SensorStatusName(us_snapshot.left.status),
    (unsigned int)us_snapshot.mid.distance_cm,
    (unsigned int)us_snapshot.mid.raw_distance_cm,
    SensorStatusName(us_snapshot.mid.status),
    (unsigned int)us_snapshot.right.distance_cm,
    (unsigned int)us_snapshot.right.raw_distance_cm,
    SensorStatusName(us_snapshot.right.status));
}
}

extern "C" void AppMain_Init(void)
{
  UartLogger &logger = UartLogger::instance();

  logger.init(&huart1);
  logger.setEnabled(1U);
  logger.setLevel(LogLevel::Debug);

  LedManager_Init();
  LedManager_SetMode(LED_MODE_BLINK_SLOW);

  g_sensor_manager.setConfig(g_sensor_cfg);
  g_sensor_manager.init();
  if (!g_sensor_manager.start())
  {
    LOGE("INIT", "SensorManager start failed");
    Error_Handler();
  }

  MotorController_InitDefault(&g_motor_controller);
  if (MotorController_Start(&g_motor_controller) != HAL_OK)
  {
    LOGW("INIT", "MotorController start failed");
  }

  g_heartbeat_tick = HAL_GetTick();
  g_drop_tick = g_heartbeat_tick;
  g_last_dropped = 0U;

  LOGI("INIT", "OOP app ready, unified sensor manager started");
}

extern "C" void AppMain_Loop(void)
{
  uint32_t now_ms = HAL_GetTick();
  SensorEventFlags events = g_sensor_manager.update(now_ms);
  const SensorManagerSnapshot &snapshot = g_sensor_manager.snapshot();
  UartLogger &logger = UartLogger::instance();

  MotorController_Update(&g_motor_controller);
  LedManager_Update();

  if ((events & SensorManager::EVENT_LINE_UPDATED) != 0U)
  {
    App_LogLineRateLimited(snapshot.line);
  }

  if ((events & SensorManager::EVENT_ULTRASONIC_UPDATED) != 0U)
  {
    App_LogUltrasonicRateLimited(snapshot.ultrasonic);
  }

  if ((now_ms - g_heartbeat_tick) >= APP_HEARTBEAT_MS)
  {
    g_heartbeat_tick = now_ms;
    LOGI(
      "HB",
      "line=%u us=%u/%u/%u led=%s",
      (unsigned int)snapshot.line.has_line,
      (unsigned int)snapshot.ultrasonic.left.distance_cm,
      (unsigned int)snapshot.ultrasonic.mid.distance_cm,
      (unsigned int)snapshot.ultrasonic.right.distance_cm,
      LedManager_ModeName(LedManager_GetMode()));
  }

  if ((now_ms - g_drop_tick) >= APP_DROP_REPORT_MS)
  {
    uint32_t dropped = logger.droppedCount();
    g_drop_tick = now_ms;

    if (dropped != g_last_dropped)
    {
      LOGW("LOG", "uart drop count=%lu", (unsigned long)dropped);
      g_last_dropped = dropped;
    }
  }

  logger.process();
}

