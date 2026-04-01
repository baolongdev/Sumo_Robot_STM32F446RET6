#include "app/app_main.h"

#include "i2c.h"
#include "modules/control/led_manager.h"
#include "modules/control/motor.h"
#include "modules/logging/uart_logger.hpp"
#include "modules/sensors/sensor_manager.hpp"
#include "tim.h"
#include "usart.h"

namespace
{
constexpr uint16_t DISTANCE_TIMEOUT_FALL_CM = 130U;
constexpr uint16_t DISTANCE_MAX_CM = 120U;
constexpr uint16_t DISTANCE_INVALID_CM = 0xFFFFU;
constexpr uint32_t KEY_RUN = 0x5501U;

/* State machine chính của robot:
   chạy thẳng tìm mục tiêu -> track -> giữ giữa -> attack,
   nhưng luôn có nhánh escape ưu tiên khi chạm line đen. */
enum class AppState : uint8_t
{
  SearchForward = 0,
  TrackLeft,
  TrackRight,
  StableCenter,
  Attack,
  EscapeBackward,
  EscapeTurn
};

enum class LastSeenDirection : uint8_t
{
  None = 0,
  Left,
  Right,
  Front
};

struct AppMainConfig_t
{
  /* Toàn bộ tham số tuning runtime được gom tại đây để dễ hiệu chỉnh ngoài sân test. */
  uint8_t motor_invert_left;
  uint8_t motor_invert_right;

  uint8_t search_forward_percent;
  uint8_t track_left_turn_percent;
  uint8_t track_right_turn_percent;
  uint8_t attack_percent;
  uint8_t backward_percent;
  uint8_t escape_turn_percent;

  uint32_t stable_attack_delay_ms;
  uint32_t escape_backward_ms;
  uint32_t escape_turn_ms;
  uint32_t lost_track_hold_ms;
  uint32_t log_interval_ms;

  uint8_t alternate_escape_turn;
  uint8_t escape_turn_right_default;

  uint16_t detect_distance_cm;
  uint16_t track_side_margin_cm;
  uint32_t track_state_hold_ms;

  SensorManagerConfig sensors;
};

struct AppRuntime_t
{
  /* Runtime chỉ giữ phần trạng thái động; mọi ngưỡng và timing nằm ở g_cfg. */
  AppState state;
  AppState prev_state;
  uint32_t state_tick_ms;
  uint8_t escape_turn_right;
  LastSeenDirection last_seen_direction;
  uint32_t last_seen_tick_ms;
};

static AppMainConfig_t g_cfg = {};
static AppRuntime_t g_rt = {AppState::SearchForward, AppState::SearchForward, 0U, 0U, LastSeenDirection::None, 0U};
static MotorController_t g_motor_controller;
static SensorManager g_sensor_manager(g_cfg.sensors);

static void AppMain_SetupConfig(void)
{
  /* Hiệu chỉnh chiều quay motor theo cách đấu dây/cơ khí hiện tại. */
  g_cfg.motor_invert_left = 1U;
  g_cfg.motor_invert_right = 0U;

  /* Tốc độ các trạng thái, tính theo %.
     - search_forward_percent: tốc độ chạy tới khi chưa thấy đối thủ
     - track_left/right_turn_percent: tốc độ xoay bám khi đối thủ lệch trái/phải
     - attack_percent: tốc độ đẩy khi đã giữ STB đủ lâu và chuyển sang ATK
     - backward_percent: tốc độ lùi khi gặp line đen
     - escape_turn_percent: tốc độ quay tránh mép sau khi đã lùi */
  g_cfg.search_forward_percent = 50U;
  g_cfg.track_left_turn_percent = 42U;
  g_cfg.track_right_turn_percent = 50U;
  g_cfg.attack_percent = 80U;
  g_cfg.backward_percent = 70U;
  g_cfg.escape_turn_percent = 56U;

  /* Thời gian cho từng trạng thái, đơn vị ms.
     - stable_attack_delay_ms: thời gian giữ đối thủ ở giữa trước khi chuyển STB -> ATK
     - escape_backward_ms: thời gian lùi khi bị ngắt bởi line đen
     - escape_turn_ms: thời gian quay tránh mép sau khi lùi
     - lost_track_hold_ms: thời gian nhớ ngắn để giữ tracking khi ultrasonic chập chờn
     - log_interval_ms: chu kỳ log UART thông thường */
  g_cfg.stable_attack_delay_ms = 500U;
  g_cfg.escape_backward_ms = 370U;
  g_cfg.escape_turn_ms = 260U;
  g_cfg.lost_track_hold_ms = 220U;
  g_cfg.log_interval_ms = 180U;

  /* Cách chọn hướng quay khi tránh mép.
     Nếu alternate_escape_turn = 1 thì khi không rõ nên quay bên nào,
     robot sẽ luân phiên trái/phải.
     Nếu = 0 thì escape_turn_right_default quyết định hướng quay mặc định. */
  g_cfg.alternate_escape_turn = 1U;
  g_cfg.escape_turn_right_default = 0U;

  /* Ngưỡng nhận diện đối thủ theo cm.
     Giá trị ultrasonic hợp lệ bị chặn trong khoảng 0..120 cm.
     TIMEOUT_FALL được map thành 130 cm, nên 130 sẽ không bị tính là có đối thủ. */
  g_cfg.detect_distance_cm = DISTANCE_MAX_CM;

  /* Tham số làm mượt tracking để xe bớt lắc trái/phải.
     - nếu sensor giữa đã thấy đối thủ thì ưu tiên giữ STB để chuẩn bị sang ATK
     - track_side_margin_cm: chỉ đổi hẳn sang trái/phải khi bên đó gần hơn rõ rệt
     - track_state_hold_ms: thời gian giữ hướng tracking tối thiểu trước khi cho đổi hướng */
  g_cfg.track_side_margin_cm = 20U;
  g_cfg.track_state_hold_ms = 240U;

  /* Cấu hình line sensor.
     active_low = 0 nghĩa là sau khi map polarity, line đen được xem là mức 1. */
  g_cfg.sensors.line.hi2c = &hi2c1;
  g_cfg.sensors.line.address_7bit = LINE_SENSOR_DEFAULT_ADDR_7BIT;
  g_cfg.sensors.line.timeout_ms = LINE_SENSOR_DEFAULT_TIMEOUT_MS;
  g_cfg.sensors.line.poll_interval_ms = 40U;
  g_cfg.sensors.line.active_low = 0U;

  /* Cấu hình timing cho ultrasonic.
     - trigger_pulse_us: độ rộng xung trigger cho HC-SR04
     - echo_rise_timeout_us: timeout chờ echo lên mức cao
     - echo_fall_timeout_us: timeout chờ echo xuống mức thấp
     - inter_sensor_gap_us: khoảng nghỉ giữa 3 cảm biến để giảm nhiễu chéo
     Hiện tại Kalman đang tắt; chỉ nên bật khi tín hiệu nhiễu và chấp nhận phản ứng chậm hơn. */
  g_cfg.sensors.ultrasonic.htim_us = &htim1;
  g_cfg.sensors.ultrasonic.trigger_pulse_us = 10U;
  g_cfg.sensors.ultrasonic.echo_rise_timeout_us = 1500U;
  g_cfg.sensors.ultrasonic.echo_fall_timeout_us = 7800U;
  g_cfg.sensors.ultrasonic.inter_sensor_gap_us = 8000U;
  g_cfg.sensors.ultrasonic.kalman_enabled = 0U;
  g_cfg.sensors.ultrasonic.kalman_q = 2.5f;
  g_cfg.sensors.ultrasonic.kalman_r = 40.0f;
}

static uint8_t App_ClampPercent(uint8_t percent)
{
  return (percent > 100U) ? 100U : percent;
}

static const char *App_StateName(AppState state)
{
  switch (state)
  {
    case AppState::SearchForward:
      return "SF";
    case AppState::TrackLeft:
      return "TL";
    case AppState::TrackRight:
      return "TR";
    case AppState::StableCenter:
      return "STB";
    case AppState::Attack:
      return "ATK";
    case AppState::EscapeBackward:
      return "ESC_B";
    case AppState::EscapeTurn:
      return "ESC_T";
    default:
      return "UNK";
  }
}

static const char *App_OpponentName(uint8_t opponent_mask)
{
  switch (opponent_mask & (SensorManager::OPPONENT_MASK_LEFT | SensorManager::OPPONENT_MASK_MID |
                           SensorManager::OPPONENT_MASK_RIGHT))
  {
    case 0U:
      return "-";
    case SensorManager::OPPONENT_MASK_LEFT:
      return "L";
    case SensorManager::OPPONENT_MASK_MID:
      return "M";
    case SensorManager::OPPONENT_MASK_RIGHT:
      return "R";
    case (SensorManager::OPPONENT_MASK_LEFT | SensorManager::OPPONENT_MASK_MID):
      return "LM";
    case (SensorManager::OPPONENT_MASK_MID | SensorManager::OPPONENT_MASK_RIGHT):
      return "MR";
    case (SensorManager::OPPONENT_MASK_LEFT | SensorManager::OPPONENT_MASK_RIGHT):
      return "LR";
    case (SensorManager::OPPONENT_MASK_LEFT | SensorManager::OPPONENT_MASK_MID |
          SensorManager::OPPONENT_MASK_RIGHT):
      return "LMR";
    default:
      return "?";
  }
}

static uint16_t App_ReportDistanceCm(const UltrasonicChannelSnapshot &channel)
{
  if (channel.distance_cm == DISTANCE_TIMEOUT_FALL_CM)
  {
    return DISTANCE_TIMEOUT_FALL_CM;
  }

  if (channel.distance_cm > DISTANCE_MAX_CM)
  {
    return DISTANCE_MAX_CM;
  }

  return channel.distance_cm;
}

static uint16_t App_TrackingDistanceCm(const UltrasonicChannelSnapshot &channel)
{
  if ((channel.valid == 0U) || (channel.status != SensorStatus::Ok))
  {
    return DISTANCE_INVALID_CM;
  }

  if (channel.distance_cm > g_cfg.detect_distance_cm)
  {
    return DISTANCE_INVALID_CM;
  }

  return channel.distance_cm;
}

static uint8_t App_LineAnyBlack(const LineSensorSnapshot &line)
{
  if ((line.valid == 0U) || (line.status != SensorStatus::Ok))
  {
    return 0U;
  }

  return (uint8_t)((line.line3 != 0U) || (line.line2 != 0U) || (line.line1 != 0U) || (line.line0 != 0U));
}

static uint8_t App_LineLeftBlack(const LineSensorSnapshot &line)
{
  if ((line.valid == 0U) || (line.status != SensorStatus::Ok))
  {
    return 0U;
  }

  return (uint8_t)((line.line3 != 0U) || (line.line2 != 0U));
}

static uint8_t App_LineRightBlack(const LineSensorSnapshot &line)
{
  if ((line.valid == 0U) || (line.status != SensorStatus::Ok))
  {
    return 0U;
  }

  return (uint8_t)((line.line1 != 0U) || (line.line0 != 0U));
}

static void App_SetState(AppState next_state, uint32_t now_ms)
{
  if (g_rt.state != next_state)
  {
    g_rt.state = next_state;
    g_rt.state_tick_ms = now_ms;
  }
}

static AppState App_SelectTrackingState(const SensorManagerSnapshot &snapshot, uint32_t now_ms, uint8_t *has_target)
{
  uint16_t left_cm = App_TrackingDistanceCm(snapshot.ultrasonic.left);
  uint16_t mid_cm = App_TrackingDistanceCm(snapshot.ultrasonic.mid);
  uint16_t right_cm = App_TrackingDistanceCm(snapshot.ultrasonic.right);
  uint8_t left_seen = (left_cm != DISTANCE_INVALID_CM) ? 1U : 0U;
  uint8_t mid_seen = (mid_cm != DISTANCE_INVALID_CM) ? 1U : 0U;
  uint8_t right_seen = (right_cm != DISTANCE_INVALID_CM) ? 1U : 0U;

  if (has_target != nullptr)
  {
    *has_target = (uint8_t)((left_seen != 0U) || (mid_seen != 0U) || (right_seen != 0U));
  }

  if ((left_seen == 0U) && (mid_seen == 0U) && (right_seen == 0U))
  {
    return g_rt.state;
  }

  /* Ưu tiên cảm biến giữa tuyệt đối: đã thấy đối thủ phía trước thì giữ STB,
     tránh rung TL/TR khi hai cảm biến bên dao động nhẹ. */
  if (mid_seen != 0U)
  {
    return AppState::StableCenter;
  }

  /* Giữ hướng đang bám trong một khoảng ngắn để không nhảy trái/phải liên tục. */
  if ((now_ms - g_rt.state_tick_ms) < g_cfg.track_state_hold_ms)
  {
    if ((g_rt.state == AppState::TrackLeft) && (left_seen != 0U))
    {
      return AppState::TrackLeft;
    }
    if ((g_rt.state == AppState::TrackRight) && (right_seen != 0U))
    {
      return AppState::TrackRight;
    }
    if ((g_rt.state == AppState::StableCenter) && (mid_seen != 0U))
    {
      return AppState::StableCenter;
    }
  }

  if ((left_seen != 0U) && (right_seen != 0U))
  {
    if (left_cm + g_cfg.track_side_margin_cm < right_cm)
    {
      return AppState::TrackLeft;
    }

    if (right_cm + g_cfg.track_side_margin_cm < left_cm)
    {
      return AppState::TrackRight;
    }

    if (mid_seen != 0U)
    {
      return AppState::StableCenter;
    }

    if (g_rt.state == AppState::TrackLeft)
    {
      return AppState::TrackLeft;
    }

    if (g_rt.state == AppState::TrackRight)
    {
      return AppState::TrackRight;
    }

    return (left_cm <= right_cm) ? AppState::TrackLeft : AppState::TrackRight;
  }

  if (left_seen != 0U)
  {
    return AppState::TrackLeft;
  }

  if (right_seen != 0U)
  {
    return AppState::TrackRight;
  }

  return AppState::StableCenter;
}

static void App_ApplySearchForwardCommand(void)
{
  MotorController_SetDriveMode(&g_motor_controller, MOTOR_DRIVE_MODE_SEARCH);
  MotorController_Forward(&g_motor_controller, App_ClampPercent(g_cfg.search_forward_percent));
}

static void App_ApplyTrackLeftCommand(void)
{
  MotorController_SetDriveMode(&g_motor_controller, MOTOR_DRIVE_MODE_CUSTOM);
  MotorController_Set(
    &g_motor_controller,
    (int16_t)-App_ClampPercent(g_cfg.track_left_turn_percent),
    (int16_t)App_ClampPercent(g_cfg.track_left_turn_percent));
}

static void App_ApplyTrackRightCommand(void)
{
  MotorController_SetDriveMode(&g_motor_controller, MOTOR_DRIVE_MODE_CUSTOM);
  MotorController_Set(
    &g_motor_controller,
    (int16_t)App_ClampPercent(g_cfg.track_right_turn_percent),
    (int16_t)-App_ClampPercent(g_cfg.track_right_turn_percent));
}

static void App_ApplyStableCenterCommand(uint32_t now_ms)
{
  /* STB chủ động dừng hẳn để tránh vừa căn mục tiêu vừa tự làm lệch tâm đo. */
  MotorController_Stop(&g_motor_controller);

  if ((now_ms - g_rt.state_tick_ms) >= g_cfg.stable_attack_delay_ms)
  {
    App_SetState(AppState::Attack, now_ms);
  }
}

static void App_ApplyAttackCommand(void)
{
  MotorController_SetDriveMode(&g_motor_controller, MOTOR_DRIVE_MODE_ATTACK);
  MotorController_Charge(&g_motor_controller, App_ClampPercent(g_cfg.attack_percent));
}

static void App_UpdateLedByState(AppState state)
{
  LedMode_t target_mode = LED_MODE_OFF;

  switch (state)
  {
    case AppState::SearchForward:
      target_mode = LED_MODE_BLINK_SLOW;
      break;

    case AppState::TrackLeft:
    case AppState::TrackRight:
      target_mode = LED_MODE_BLINK_FAST;
      break;

    case AppState::StableCenter:
      LedManager_SetCustomBlink(220U);
      target_mode = LED_MODE_BLINK_CUSTOM;
      break;

    case AppState::Attack:
      target_mode = LED_MODE_ON;
      break;

    case AppState::EscapeBackward:
    case AppState::EscapeTurn:
      LedManager_SetCustomBlink(90U);
      target_mode = LED_MODE_BLINK_CUSTOM;
      break;

    default:
      target_mode = LED_MODE_OFF;
      break;
  }

  if (LedManager_GetMode() != target_mode)
  {
    LedManager_SetMode(target_mode);
  }
}

static void App_TriggerEscape(const LineSensorSnapshot &line, uint32_t now_ms)
{
  uint8_t left_black = App_LineLeftBlack(line);
  uint8_t right_black = App_LineRightBlack(line);

  if ((left_black != 0U) && (right_black == 0U))
  {
    g_rt.escape_turn_right = 1U;
  }
  else if ((right_black != 0U) && (left_black == 0U))
  {
    g_rt.escape_turn_right = 0U;
  }
  else if (g_cfg.alternate_escape_turn != 0U)
  {
    g_rt.escape_turn_right ^= 1U;
  }
  else
  {
    g_rt.escape_turn_right = (g_cfg.escape_turn_right_default != 0U) ? 1U : 0U;
  }

  App_SetState(AppState::EscapeBackward, now_ms);
}

static void App_UpdateLastSeenDirection(AppState tracking_state, uint32_t now_ms)
{
  switch (tracking_state)
  {
    case AppState::TrackLeft:
      g_rt.last_seen_direction = LastSeenDirection::Left;
      break;

    case AppState::TrackRight:
      g_rt.last_seen_direction = LastSeenDirection::Right;
      break;

    case AppState::StableCenter:
    case AppState::Attack:
      g_rt.last_seen_direction = LastSeenDirection::Front;
      break;

    default:
      return;
  }

  g_rt.last_seen_tick_ms = now_ms;
}

static void App_UpdateBehavior(uint32_t now_ms)
{
  const SensorManagerSnapshot &snapshot = g_sensor_manager.snapshot();
  uint8_t boundary_black = App_LineAnyBlack(snapshot.line);
  uint8_t has_target = 0U;

  /* Mép sàn luôn có ưu tiên cao nhất: đang track hay attack vẫn phải cắt sang escape ngay. */
  if ((boundary_black != 0U) &&
      (g_rt.state != AppState::EscapeBackward) &&
      (g_rt.state != AppState::EscapeTurn))
  {
    App_TriggerEscape(snapshot.line, now_ms);
  }

  if (g_rt.state == AppState::EscapeBackward)
  {
    uint32_t elapsed_ms = now_ms - g_rt.state_tick_ms;

    MotorController_SetDriveMode(&g_motor_controller, MOTOR_DRIVE_MODE_ATTACK);
    MotorController_Backward(&g_motor_controller, App_ClampPercent(g_cfg.backward_percent));

    if (elapsed_ms >= g_cfg.escape_backward_ms)
    {
      App_SetState(AppState::EscapeTurn, now_ms);
    }
    return;
  }

  if (g_rt.state == AppState::EscapeTurn)
  {
    uint32_t elapsed_ms = now_ms - g_rt.state_tick_ms;

    MotorController_SetDriveMode(&g_motor_controller, MOTOR_DRIVE_MODE_ATTACK);
    if (g_rt.escape_turn_right != 0U)
    {
      MotorController_TurnRight(&g_motor_controller, App_ClampPercent(g_cfg.escape_turn_percent));
    }
    else
    {
      MotorController_TurnLeft(&g_motor_controller, App_ClampPercent(g_cfg.escape_turn_percent));
    }

    if (elapsed_ms >= g_cfg.escape_turn_ms)
    {
      App_SetState(AppState::SearchForward, now_ms);
    }
    return;
  }

  if (g_sensor_manager.hasOpponentWithinCm(g_cfg.detect_distance_cm))
  {
    /* Từ đây trở xuống là nhánh "thấy đối thủ": chọn hướng track, giữ STB, rồi mới sang ATK. */
    AppState tracking_state = App_SelectTrackingState(snapshot, now_ms, &has_target);

    if (has_target != 0U)
    {
      App_UpdateLastSeenDirection(tracking_state, now_ms);
    }

    if (g_rt.state == AppState::Attack)
    {
      App_SetState(AppState::Attack, now_ms);
    }
    else
    {
      App_SetState(tracking_state, now_ms);
    }
  }
  else if (g_rt.state == AppState::Attack)
  {
    App_SetState(AppState::SearchForward, now_ms);
  }
  else if ((now_ms - g_rt.last_seen_tick_ms) <= g_cfg.lost_track_hold_ms)
  {
    switch (g_rt.last_seen_direction)
    {
      case LastSeenDirection::Left:
        App_SetState(AppState::TrackLeft, now_ms);
        break;
      case LastSeenDirection::Right:
        App_SetState(AppState::TrackRight, now_ms);
        break;
      case LastSeenDirection::Front:
        App_SetState(AppState::SearchForward, now_ms);
        break;
      case LastSeenDirection::None:
      default:
        App_SetState(AppState::SearchForward, now_ms);
        break;
    }
  }
  else
  {
    g_rt.last_seen_direction = LastSeenDirection::None;
    if ((g_rt.state == AppState::TrackLeft) ||
        (g_rt.state == AppState::TrackRight) ||
        (g_rt.state == AppState::StableCenter))
    {
      App_SetState(AppState::SearchForward, now_ms);
    }
  }

  switch (g_rt.state)
  {
    case AppState::SearchForward:
      App_ApplySearchForwardCommand();
      break;

    case AppState::TrackLeft:
      App_ApplyTrackLeftCommand();
      break;

    case AppState::TrackRight:
      App_ApplyTrackRightCommand();
      break;

    case AppState::StableCenter:
      App_ApplyStableCenterCommand(now_ms);
      break;

    case AppState::Attack:
      App_ApplyAttackCommand();
      break;

    default:
      break;
  }
}

static void App_LogRun(UartLogger &logger)
{
  const SensorManagerSnapshot &snapshot = g_sensor_manager.snapshot();
  uint8_t opponent_mask = g_sensor_manager.opponentMaskWithinCm(g_cfg.detect_distance_cm);
  uint8_t boundary_black = App_LineAnyBlack(snapshot.line);
  uint8_t state_changed = (uint8_t)(g_rt.state != g_rt.prev_state);
  const char *state_name = App_StateName(g_rt.state);
  const char *opponent_name = App_OpponentName(opponent_mask);
  unsigned int left_cm = (unsigned int)App_ReportDistanceCm(snapshot.ultrasonic.left);
  unsigned int mid_cm = (unsigned int)App_ReportDistanceCm(snapshot.ultrasonic.mid);
  unsigned int right_cm = (unsigned int)App_ReportDistanceCm(snapshot.ultrasonic.right);
  unsigned int line3 = (unsigned int)snapshot.line.line3;
  unsigned int line2 = (unsigned int)snapshot.line.line2;
  unsigned int line1 = (unsigned int)snapshot.line.line1;
  unsigned int line0 = (unsigned int)snapshot.line.line0;

  if (state_changed != 0U)
  {
    logger.log(
      LogLevel::Info,
      "APP",
      "-> %s U=%u/%u/%u O=%s B=%u L=%u%u%u%u",
      state_name,
      left_cm,
      mid_cm,
      right_cm,
      opponent_name,
      (unsigned int)boundary_black,
      line3,
      line2,
      line1,
      line0);
  }
  else
  {
    logger.logRateLimited(
      LogLevel::Info,
      "APP",
      KEY_RUN,
      g_cfg.log_interval_ms,
      "%s U=%u/%u/%u O=%s B=%u L=%u%u%u%u",
      state_name,
      left_cm,
      mid_cm,
      right_cm,
      opponent_name,
      (unsigned int)boundary_black,
      line3,
      line2,
      line1,
      line0);
  }

  g_rt.prev_state = g_rt.state;
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
  LedManager_SetMode(LED_MODE_OFF);

  g_sensor_manager.setConfig(g_cfg.sensors);
  g_sensor_manager.init();
  if (!g_sensor_manager.start())
  {
    LOGE("INIT", "Sensor manager start failed");
    Error_Handler();
  }

  MotorController_InitDefault(&g_motor_controller);
  MotorController_SetInversion(&g_motor_controller, g_cfg.motor_invert_left, g_cfg.motor_invert_right);
  MotorController_SetStopMode(&g_motor_controller, MOTOR_STOP_BRAKE);
  MotorController_SetDirectionSafety(&g_motor_controller, 80U, 40U);
  MotorController_SetMinDrivePercent(&g_motor_controller, 18U, 10U);
  MotorController_SetCommandTimeout(&g_motor_controller, 600U);
  if (MotorController_Start(&g_motor_controller) != HAL_OK)
  {
    LOGE("INIT", "Motor controller start failed");
    Error_Handler();
  }

  g_rt.state = AppState::SearchForward;
  g_rt.prev_state = AppState::SearchForward;
  g_rt.state_tick_ms = HAL_GetTick();
  g_rt.escape_turn_right = (g_cfg.escape_turn_right_default != 0U) ? 1U : 0U;
  g_rt.last_seen_direction = LastSeenDirection::None;
  g_rt.last_seen_tick_ms = 0U;

  App_UpdateLedByState(g_rt.state);
  LOGI("INIT", "Search-forward/track/attack mode started with line interrupt priority");
}

extern "C" void AppMain_Loop(void)
{
  uint32_t now_ms = HAL_GetTick();
  UartLogger &logger = UartLogger::instance();

  (void)g_sensor_manager.update(now_ms);
  App_UpdateBehavior(now_ms);
  App_UpdateLedByState(g_rt.state);
  MotorController_Update(&g_motor_controller);
  App_LogRun(logger);

  LedManager_Update();
  logger.process();
}

extern "C" void AppMain_SetDriveCommand(AppDriveCommand_t command, uint8_t speed_percent)
{
  uint8_t speed = App_ClampPercent(speed_percent);

  MotorController_SetDriveMode(&g_motor_controller, MOTOR_DRIVE_MODE_CUSTOM);
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
