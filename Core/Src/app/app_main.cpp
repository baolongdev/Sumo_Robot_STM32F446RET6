#include "app/app_main.h"

#include "i2c.h"
#include "modules/control/led_manager.h"
#include "modules/control/motor.h"
#include "modules/logging/uart_logger.hpp"
#include "modules/sensors/line_sensor.h"
#include "usart.h"

namespace
{
constexpr uint32_t KEY_RUN = 0x4403U;
constexpr uint32_t RUN_LOG_INTERVAL_MS = 250U;

struct AppMainConfig_t
{
  uint8_t motor_invert_left;
  uint8_t motor_invert_right;

  uint8_t speed_forward_percent;
  uint8_t speed_backward_percent;
  uint8_t speed_turn_percent;

  uint32_t black_react_backward_ms;
  uint32_t black_react_turn_ms;
  uint8_t turn_right_default;
  uint8_t turn_alternate_on_each_black;

  uint8_t line_addr_7bit;
  LineSensorPolarity_t line_polarity;
  uint32_t line_poll_ms;
};

enum class AppRunState : uint8_t
{
  Forward = 0,
  BackwardOnBlack,
  TurnOnBlack
};

struct AppRuntime_t
{
  AppRunState state;
  AppRunState prev_state;
  uint32_t state_tick_ms;
  uint8_t turn_right_now;
  uint8_t prev_line_black;
};

struct LineFrame_t
{
  const LineSensorSnapshot_t *snapshot;
  uint8_t updated;
  uint8_t black;
  uint8_t edge_black;
};

static MotorController_t g_motor_controller;
static LineSensor_t g_line_sensor;
static AppMainConfig_t g_cfg = {};
static AppRuntime_t g_rt = {AppRunState::Forward, AppRunState::Forward, 0U, 0U, 0U};

static void AppMain_SetupConfig(void)
{
  /* USER SETUP: chỉ sửa thông số tại đây */
  g_cfg.motor_invert_left = 1U;
  g_cfg.motor_invert_right = 0U;

  g_cfg.speed_forward_percent = 30U;
  g_cfg.speed_backward_percent = 60U;
  g_cfg.speed_turn_percent = 60U;

  g_cfg.black_react_backward_ms = 200U;
  g_cfg.black_react_turn_ms = 500U;
  g_cfg.turn_right_default = 0U;            /* 0: quay trái, 1: quay phải */
  g_cfg.turn_alternate_on_each_black = 1U;  /* 1: trái/phải luân phiên */

  g_cfg.line_addr_7bit = LINE_SENSOR_DEFAULT_ADDR_7BIT;
  g_cfg.line_polarity = LINE_SENSOR_POLARITY_ACTIVE_HIGH; /* 1 = đen, 0 = không đen */
  g_cfg.line_poll_ms = LINE_SENSOR_DEFAULT_POLL_MS;
}

static uint8_t App_ClampSpeed(uint8_t speed_percent)
{
  if (speed_percent > 100U)
  {
    return 100U;
  }
  return speed_percent;
}

static const char *App_RunStateName(AppRunState state)
{
  switch (state)
  {
    case AppRunState::Forward:
      return "FORWARD";
    case AppRunState::BackwardOnBlack:
      return "BACKWARD";
    case AppRunState::TurnOnBlack:
      return "TURN";
    default:
      return "UNKNOWN";
  }
}

static uint8_t App_LineAnyBlack(const LineSensorSnapshot_t *snapshot)
{
  if ((snapshot == nullptr) || (snapshot->status != HAL_OK))
  {
    return 0U;
  }

  return (uint8_t)(
    (snapshot->line3 != 0U) ||
    (snapshot->line2 != 0U) ||
    (snapshot->line1 != 0U) ||
    (snapshot->line0 != 0U));
}

static LineFrame_t App_ReadLineFrame(uint32_t now_ms)
{
  LineFrame_t frame;

  frame.updated = LineSensor_Update(&g_line_sensor, now_ms);
  frame.snapshot = LineSensor_GetSnapshot(&g_line_sensor);
  frame.black = App_LineAnyBlack(frame.snapshot);
  frame.edge_black = (uint8_t)((frame.black != 0U) && (g_rt.prev_line_black == 0U));
  g_rt.prev_line_black = frame.black;
  return frame;
}

static void App_SetForwardCommand(void)
{
  MotorController_SetDriveMode(&g_motor_controller, MOTOR_DRIVE_MODE_SEARCH);
  MotorController_Forward(&g_motor_controller, g_cfg.speed_forward_percent);
}

static void App_SetBackwardCommand(void)
{
  MotorController_SetDriveMode(&g_motor_controller, MOTOR_DRIVE_MODE_SEARCH);
  MotorController_Backward(&g_motor_controller, g_cfg.speed_backward_percent);
}

static void App_SetTurnCommand(void)
{
  MotorController_SetDriveMode(&g_motor_controller, MOTOR_DRIVE_MODE_SEARCH);
  if (g_rt.turn_right_now != 0U)
  {
    MotorController_TurnRight(&g_motor_controller, g_cfg.speed_turn_percent);
  }
  else
  {
    MotorController_TurnLeft(&g_motor_controller, g_cfg.speed_turn_percent);
  }
}

static void App_TriggerBlackInterrupt(uint32_t now_ms)
{
  if (g_cfg.turn_alternate_on_each_black != 0U)
  {
    g_rt.turn_right_now ^= 1U;
  }
  else
  {
    g_rt.turn_right_now = (g_cfg.turn_right_default != 0U) ? 1U : 0U;
  }

  g_rt.state = AppRunState::BackwardOnBlack;
  g_rt.state_tick_ms = now_ms;
}

static void App_UpdateAutoRun(uint32_t now_ms)
{
  uint32_t elapsed_ms = now_ms - g_rt.state_tick_ms;

  switch (g_rt.state)
  {
    case AppRunState::Forward:
      App_SetForwardCommand();
      break;

    case AppRunState::BackwardOnBlack:
      App_SetBackwardCommand();
      if (elapsed_ms >= g_cfg.black_react_backward_ms)
      {
        g_rt.state = AppRunState::TurnOnBlack;
        g_rt.state_tick_ms = now_ms;
      }
      break;

    case AppRunState::TurnOnBlack:
    default:
      App_SetTurnCommand();
      if (elapsed_ms >= g_cfg.black_react_turn_ms)
      {
        g_rt.state = AppRunState::Forward;
        g_rt.state_tick_ms = now_ms;
      }
      break;
  }
}

static void App_LogDebug(UartLogger &logger, const LineFrame_t &frame, uint8_t irq_black, uint8_t state_changed)
{
  uint8_t line_changed = 0U;
  if ((frame.updated != 0U) && (frame.snapshot != nullptr))
  {
    line_changed = (uint8_t)((frame.snapshot->event_flags & (LINE_SENSOR_EVENT_STATE_CHANGED | LINE_SENSOR_EVENT_STATUS_CHANGED)) != 0U);
  }

  if ((irq_black != 0U) || (state_changed != 0U) || (line_changed != 0U))
  {
    logger.log(
      LogLevel::Info,
      "DBG",
      "st=%s irq=%u edge=%u black=%u L=%u%u%u%u ls=%s upd=%u",
      App_RunStateName(g_rt.state),
      (unsigned int)irq_black,
      (unsigned int)frame.edge_black,
      (unsigned int)frame.black,
      (unsigned int)((frame.snapshot != nullptr) ? frame.snapshot->line3 : 0U),
      (unsigned int)((frame.snapshot != nullptr) ? frame.snapshot->line2 : 0U),
      (unsigned int)((frame.snapshot != nullptr) ? frame.snapshot->line1 : 0U),
      (unsigned int)((frame.snapshot != nullptr) ? frame.snapshot->line0 : 0U),
      LineSensor_StatusName((frame.snapshot != nullptr) ? frame.snapshot->status : HAL_ERROR),
      (unsigned int)frame.updated);
    return;
  }

  logger.logRateLimited(
    LogLevel::Info,
    "DBG",
    KEY_RUN,
    RUN_LOG_INTERVAL_MS,
    "st=%s irq=%u edge=%u black=%u L=%u%u%u%u ls=%s upd=%u",
    App_RunStateName(g_rt.state),
    (unsigned int)irq_black,
    (unsigned int)frame.edge_black,
    (unsigned int)frame.black,
    (unsigned int)((frame.snapshot != nullptr) ? frame.snapshot->line3 : 0U),
    (unsigned int)((frame.snapshot != nullptr) ? frame.snapshot->line2 : 0U),
    (unsigned int)((frame.snapshot != nullptr) ? frame.snapshot->line1 : 0U),
    (unsigned int)((frame.snapshot != nullptr) ? frame.snapshot->line0 : 0U),
    LineSensor_StatusName((frame.snapshot != nullptr) ? frame.snapshot->status : HAL_ERROR),
    (unsigned int)frame.updated);
}
}

extern "C" void AppMain_Init(void)
{
  UartLogger &logger = UartLogger::instance();
  AppMain_SetupConfig();

  logger.init(&huart1);
  logger.setEnabled(1U);
  logger.setLevel(LogLevel::Info);

  LedManager_Init();
  LedManager_SetMode(LED_MODE_ON);

  MotorController_InitDefault(&g_motor_controller);
  MotorController_SetInversion(&g_motor_controller, g_cfg.motor_invert_left, g_cfg.motor_invert_right);
  MotorController_SetCommandTimeout(&g_motor_controller, 2000U);
  if (MotorController_Start(&g_motor_controller) != HAL_OK)
  {
    LOGE("INIT", "MotorController start failed");
    Error_Handler();
  }

  LineSensor_Init(&g_line_sensor, &hi2c1, g_cfg.line_addr_7bit);
  LineSensor_SetPolarity(&g_line_sensor, g_cfg.line_polarity);
  LineSensor_SetPollInterval(&g_line_sensor, g_cfg.line_poll_ms);
  LineSensor_ResetScheduler(&g_line_sensor, HAL_GetTick());

  g_rt.state = AppRunState::Forward;
  g_rt.prev_state = AppRunState::Forward;
  g_rt.turn_right_now = (g_cfg.turn_right_default != 0U) ? 1U : 0U;
  g_rt.prev_line_black = 0U;
  g_rt.state_tick_ms = HAL_GetTick();

  LOGI("INIT", "Auto run + soft IRQ line den: gap line den -> lui theo thoi gian -> quay -> tien");
}

extern "C" void AppMain_Loop(void)
{
  uint32_t now_ms = HAL_GetTick();
  UartLogger &logger = UartLogger::instance();
  LineFrame_t frame = App_ReadLineFrame(now_ms);
  uint8_t irq_black = 0U;
  uint8_t state_changed;

  if ((frame.black != 0U) && (g_rt.state == AppRunState::Forward))
  {
    /* Đang FORWARD mà thấy line đen thì ngắt luồng hiện tại, lùi ngay. */
    App_TriggerBlackInterrupt(now_ms);
    irq_black = 1U;
  }

  App_UpdateAutoRun(now_ms);
  MotorController_Update(&g_motor_controller);

  state_changed = (uint8_t)(g_rt.state != g_rt.prev_state);
  g_rt.prev_state = g_rt.state;

  App_LogDebug(logger, frame, irq_black, state_changed);

  LedManager_Update();
  logger.process();
}

extern "C" void AppMain_SetDriveCommand(AppDriveCommand_t command, uint8_t speed_percent)
{
  uint8_t speed = App_ClampSpeed(speed_percent);

  MotorController_SetDriveMode(&g_motor_controller, MOTOR_DRIVE_MODE_SEARCH);
  switch (command)
  {
    case APP_DRIVE_FORWARD:
      MotorController_Forward(&g_motor_controller, speed);
      break;

    case APP_DRIVE_BACKWARD:
      MotorController_Backward(&g_motor_controller, speed);
      break;

    case APP_DRIVE_TURN_LEFT:
      MotorController_TurnLeft(&g_motor_controller, speed);
      break;

    case APP_DRIVE_TURN_RIGHT:
      MotorController_TurnRight(&g_motor_controller, speed);
      break;

    case APP_DRIVE_STOP:
    default:
      MotorController_Stop(&g_motor_controller);
      break;
  }
}

extern "C" void AppMain_Forward(uint8_t speed_percent)
{
  AppMain_SetDriveCommand(APP_DRIVE_FORWARD, speed_percent);
}

extern "C" void AppMain_Backward(uint8_t speed_percent)
{
  AppMain_SetDriveCommand(APP_DRIVE_BACKWARD, speed_percent);
}

extern "C" void AppMain_TurnLeft(uint8_t speed_percent)
{
  AppMain_SetDriveCommand(APP_DRIVE_TURN_LEFT, speed_percent);
}

extern "C" void AppMain_TurnRight(uint8_t speed_percent)
{
  AppMain_SetDriveCommand(APP_DRIVE_TURN_RIGHT, speed_percent);
}

extern "C" void AppMain_Stop(void)
{
  AppMain_SetDriveCommand(APP_DRIVE_STOP, 0U);
}
