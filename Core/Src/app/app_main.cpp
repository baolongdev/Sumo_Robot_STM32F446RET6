#include "app/app_main.h"

#include "gpio.h"
#include "i2c.h"
#include "modules/control/led_manager.h"
#include "modules/control/motor.h"
#include "modules/logging/uart_logger.hpp"
#include "modules/sensors/sensor_manager.hpp"
#include "tim.h"
#include "usart.h"

namespace
{
/*
 * Luồng điều khiển tổng quát (chế độ chỉ quay):
 * 1) Bám hướng đối thủ bằng L/R ultrasonic -> TURN_LEFT / TURN_RIGHT / LOCK
 * 2) Mất mục tiêu -> chạy SEARCH pattern (trái nhẹ, phải nhẹ, trái mạnh lặp lại)
 * 3) Gặp line đen -> EDGE_ESCAPE ưu tiên cao nhất:
 *    lùi nhẹ về vùng an toàn, quay né ngẫu nhiên, rồi quay lại SEARCH.
 */
constexpr uint32_t APP_HEARTBEAT_MS = 0U;
constexpr uint32_t APP_STATE_LOG_MS = 90U;
constexpr uint32_t APP_DROP_REPORT_MS = 2000U;

constexpr uint32_t KEY_STATE = 0x1010U;

/* Ngưỡng căn hướng (cm). */
constexpr uint16_t TURN_MARGIN_ON_CM = 4U;
constexpr uint16_t TURN_MARGIN_OFF_CM = 2U;
constexpr uint16_t SIDE_TRACK_MAX_CM = 120U;
constexpr uint16_t OPPONENT_DETECT_ON_CM = 28U;

/* Kiểm tra hợp lệ + lọc nhiễu đầu vào. */
/* Raw phải nằm trong dải hợp lệ và không nhảy quá xa so với giá trị đã lọc. */
constexpr uint16_t US_VALID_MIN_CM = 2U;
constexpr uint16_t US_VALID_MAX_CM = 200U;
constexpr uint16_t US_OUTLIER_DELTA_CM = 120U;
constexpr uint8_t FILTER_ALPHA_NUM = 5U;  /* 0.5 */
constexpr uint8_t FILTER_ALPHA_DEN = 10U;
constexpr uint32_t ULTRASONIC_STALE_MS = 250U;
constexpr uint32_t TARGET_LOST_MS = 250U;

/* Ổn định chuyển trạng thái. */
/* Trạng thái ứng viên phải ổn định N chu kỳ mới được chấp nhận. */
constexpr uint8_t STATE_CONFIRM_CYCLES = 3U;
constexpr uint32_t STATE_MIN_HOLD_MS = 220U;

/* Tốc độ motor mặc định (%). */
constexpr uint8_t SPD_TURN = 45U;
constexpr uint8_t SPD_SEARCH_SOFT = 28U;
constexpr uint8_t SPD_SEARCH_HARD = 55U;
constexpr uint32_t SEARCH_SOFT_WINDOW_MS = 260U;
constexpr uint32_t SEARCH_HARD_WINDOW_MS = 520U;

constexpr uint8_t MOTOR_INVERT_LEFT = 0U;
constexpr uint8_t MOTOR_INVERT_RIGHT = 0U;

/* Né biên (line đen): lùi nhẹ rồi quay ngẫu nhiên. */
constexpr uint8_t EDGE_BACKWARD_SPEED = 30U;
constexpr uint8_t EDGE_TURN_SPEED = 45U;
constexpr uint32_t EDGE_BACKWARD_MS = 140U;
constexpr uint32_t EDGE_TURN_MIN_MS = 220U;
constexpr uint32_t EDGE_TURN_MAX_MS = 620U;
constexpr uint32_t LED_SEARCH_PERIOD_MS = 760U;
constexpr uint32_t LED_TURN_LEFT_PERIOD_MS = 120U;
constexpr uint32_t LED_TURN_RIGHT_PERIOD_MS = 340U;
constexpr uint32_t LED_EDGE_PERIOD_MS = 70U;

static SensorManagerConfig g_sensor_cfg = {
  /* Cấu hình line sensor (I2C): địa chỉ, timeout, chu kỳ poll, polarity. */
  {
    &hi2c1,
    LINE_SENSOR_DEFAULT_ADDR_7BIT,
    LINE_SENSOR_DEFAULT_TIMEOUT_MS,
    LINE_SENSOR_DEFAULT_POLL_MS,
    1U
  },
  /* Cấu hình ultrasonic: TIM đo us, trigger, timeout rise/fall, gap giữa các cảm biến. */
  {
    &htim1,
    10U,
    8000U,
    35000U,
    12000U,
    0U,
    12.0f,
    16.0f
  }
};

static SensorManager g_sensor_manager(g_sensor_cfg);
static MotorController_t g_motor_controller;

static uint32_t g_heartbeat_tick = 0U;
static uint32_t g_drop_tick = 0U;
static uint32_t g_last_dropped = 0U;

enum class RobotState : uint8_t
{
  Search = 0,
  TurnLeft,
  TurnRight,
  Lock,
  EdgeEscape
};

enum class SearchPhase : uint8_t
{
  SoftLeft = 0,
  SoftRight,
  HardLeft
};

enum class EdgePhase : uint8_t
{
  Idle = 0,
  Backward,
  Turn
};

struct UsFilterChannel
{
  /* Lịch sử 3 mẫu cho bộ lọc median */
  uint16_t hist[3];
  uint8_t hist_count;
  uint8_t hist_index;
  uint8_t initialized;
  uint8_t valid;
  uint16_t filtered_cm;
  uint32_t last_sample_tick_ms;
  uint32_t last_valid_tick_ms;
};

struct UsFilterState
{
  UsFilterChannel left;
  UsFilterChannel mid;
  UsFilterChannel right;
};

struct TargetView
{
  uint16_t left_cm;
  uint16_t mid_cm;
  uint16_t right_cm;
  uint8_t left_valid;
  uint8_t mid_valid;
  uint8_t right_valid;
};

struct RobotSmContext
{
  /* Trạng thái runtime + bộ nhớ chống rung khi chuyển state */
  RobotState state;
  RobotState prev_state;
  RobotState pending_state;
  uint8_t state_changed;
  uint8_t pending_count;
  uint8_t turn_left_latch;
  uint8_t turn_right_latch;
  uint8_t opponent_latch;
  uint32_t state_tick_ms;
  uint32_t last_target_tick_ms;
  uint32_t search_phase_tick_ms;
  SearchPhase search_phase;
};

struct EdgeEscapeContext
{
  /* Mini state machine cho né biên */
  uint8_t active;
  EdgePhase phase;
  int8_t turn_dir; /* -1: quay trái, +1: quay phải */
  uint32_t phase_tick_ms;
  uint32_t turn_duration_ms;
};

static UsFilterState g_us_filter = {};
static EdgeEscapeContext g_edge = {0U, EdgePhase::Idle, 0, 0U, 0U};
static uint32_t g_rand_state = 0x6D2B79F5UL;

static RobotSmContext g_sm = {
  RobotState::Search,
  RobotState::Search,
  RobotState::Search,
  0U,
  0U,
  0U,
  0U,
  0U,
  0U,
  0U,
  0U,
  SearchPhase::SoftLeft
};

static const char *App_StateName(RobotState state)
{
  switch (state)
  {
    case RobotState::Search:
      return "SEARCH";
    case RobotState::TurnLeft:
      return "TURN_LEFT";
    case RobotState::TurnRight:
      return "TURN_RIGHT";
    case RobotState::Lock:
      return "LOCK";
    case RobotState::EdgeEscape:
      return "EDGE_ESCAPE";
    default:
      return "UNKNOWN";
  }
}

static const char *App_SearchPhaseName(SearchPhase phase)
{
  switch (phase)
  {
    case SearchPhase::SoftLeft:
      return "SOFT_L";
    case SearchPhase::SoftRight:
      return "SOFT_R";
    case SearchPhase::HardLeft:
      return "HARD_L";
    default:
      return "UNK";
  }
}

static const char *App_EdgePhaseName(EdgePhase phase)
{
  switch (phase)
  {
    case EdgePhase::Idle:
      return "IDLE";
    case EdgePhase::Backward:
      return "BACK";
    case EdgePhase::Turn:
      return "TURN";
    default:
      return "UNK";
  }
}

static uint16_t App_AbsDiffU16(uint16_t a, uint16_t b)
{
  return (a > b) ? (uint16_t)(a - b) : (uint16_t)(b - a);
}

static uint32_t App_Rand32(void)
{
  /* Bộ sinh số giả ngẫu nhiên LCG nhỏ gọn (không dùng cấp phát động). */
  g_rand_state = (1664525UL * g_rand_state) + 1013904223UL;
  return g_rand_state;
}

static uint32_t App_RandRange(uint32_t min_v, uint32_t max_v)
{
  uint32_t span;

  if (max_v <= min_v)
  {
    return min_v;
  }

  span = max_v - min_v + 1U;
  return min_v + (App_Rand32() % span);
}

static uint16_t App_Median3(uint16_t a, uint16_t b, uint16_t c)
{
  uint16_t t;

  if (a > b)
  {
    t = a;
    a = b;
    b = t;
  }
  if (b > c)
  {
    t = b;
    b = c;
    c = t;
  }
  if (a > b)
  {
    t = a;
    a = b;
    b = t;
  }
  return b;
}

static uint16_t App_FilteredIir(uint16_t prev, uint16_t sample)
{
  uint32_t acc = ((uint32_t)FILTER_ALPHA_NUM * sample) +
                 ((uint32_t)(FILTER_ALPHA_DEN - FILTER_ALPHA_NUM) * prev) +
                 (FILTER_ALPHA_DEN / 2U);
  return (uint16_t)(acc / FILTER_ALPHA_DEN);
}

/**
 * @brief Kiểm tra mẫu ultrasonic raw có hợp lệ để dùng cho lọc hay không.
 * @param channel  Snapshot 1 kênh ultrasonic (left/mid/right).
 * @param out_raw_cm Con trỏ nhận giá trị raw (cm) khi hợp lệ.
 * @return 1 nếu hợp lệ, 0 nếu không hợp lệ (timeout/status lỗi/ngoài dải).
 */
static uint8_t App_ReadPlausibleRaw(const UltrasonicChannelSnapshot &channel, uint16_t *out_raw_cm)
{
  uint16_t raw_cm;

  if ((out_raw_cm == nullptr) || (channel.status != SensorStatus::Ok))
  {
    return 0U;
  }

  raw_cm = channel.raw_distance_cm;
  if ((raw_cm < US_VALID_MIN_CM) || (raw_cm > US_VALID_MAX_CM))
  {
    return 0U;
  }

  *out_raw_cm = raw_cm;
  return 1U;
}

/**
 * @brief Cập nhật bộ lọc cho 1 kênh ultrasonic (median + IIR + stale check).
 * @param source Snapshot mới từ SensorManager của kênh đang xét.
 * @param channel Trạng thái bộ lọc nội bộ của kênh.
 * @param now_ms Tick hiện tại (HAL_GetTick).
 *
 * Quy trình:
 * 1) Bỏ qua nếu chưa có mẫu mới.
 * 2) Kiểm tra hợp lệ + reject outlier.
 * 3) Cập nhật median-3, sau đó làm mượt IIR.
 * 4) Đánh dấu valid nếu dữ liệu còn "tươi" trong cửa sổ ULTRASONIC_STALE_MS.
 */
static void App_UpdateUsFilterChannel(const UltrasonicChannelSnapshot &source, UsFilterChannel *channel, uint32_t now_ms)
{
  uint16_t raw_cm = 0U;
  uint8_t accepted;
  uint16_t median_cm;

  if (channel == nullptr)
  {
    return;
  }

  if (source.tick_ms == channel->last_sample_tick_ms)
  {
    channel->valid = ((channel->initialized != 0U) && ((now_ms - channel->last_valid_tick_ms) <= ULTRASONIC_STALE_MS)) ? 1U : 0U;
    return;
  }

  channel->last_sample_tick_ms = source.tick_ms;
  accepted = App_ReadPlausibleRaw(source, &raw_cm);

  /* Loại bỏ bước nhảy đột ngột không thực tế (spike ultrasonic). */
  if ((accepted != 0U) && (channel->initialized != 0U) &&
      (App_AbsDiffU16(raw_cm, channel->filtered_cm) > US_OUTLIER_DELTA_CM))
  {
    accepted = 0U;
  }

  if (accepted != 0U)
  {
    channel->hist[channel->hist_index] = raw_cm;
    channel->hist_index = (uint8_t)((channel->hist_index + 1U) % 3U);
    if (channel->hist_count < 3U)
    {
      channel->hist_count++;
    }

    /* Median-3 (hoặc trung bình cho 2 mẫu đầu), sau đó làm mượt IIR. */
    if (channel->hist_count == 1U)
    {
      median_cm = channel->hist[0];
    }
    else if (channel->hist_count == 2U)
    {
      median_cm = (uint16_t)((channel->hist[0] + channel->hist[1]) / 2U);
    }
    else
    {
      median_cm = App_Median3(channel->hist[0], channel->hist[1], channel->hist[2]);
    }

    if (channel->initialized == 0U)
    {
      channel->filtered_cm = median_cm;
      channel->initialized = 1U;
    }
    else
    {
      channel->filtered_cm = App_FilteredIir(channel->filtered_cm, median_cm);
    }
    channel->last_valid_tick_ms = now_ms;
  }

  channel->valid = ((channel->initialized != 0U) && ((now_ms - channel->last_valid_tick_ms) <= ULTRASONIC_STALE_MS)) ? 1U : 0U;
}

/**
 * @brief Cập nhật bộ lọc cho cả 3 kênh ultrasonic.
 * @param snapshot Snapshot tổng từ SensorManager.
 * @param now_ms Tick hiện tại (HAL_GetTick).
 */
static void App_UpdateUsFilter(const SensorManagerSnapshot &snapshot, uint32_t now_ms)
{
  App_UpdateUsFilterChannel(snapshot.ultrasonic.left, &g_us_filter.left, now_ms);
  App_UpdateUsFilterChannel(snapshot.ultrasonic.mid, &g_us_filter.mid, now_ms);
  App_UpdateUsFilterChannel(snapshot.ultrasonic.right, &g_us_filter.right, now_ms);
}

/**
 * @brief Tạo "khung nhìn mục tiêu" đã lọc để state machine dùng.
 * @return TargetView gồm {left/mid/right cm} và cờ valid tương ứng.
 */
static TargetView App_BuildTargetView(void)
{
  TargetView view;

  view.left_cm = g_us_filter.left.filtered_cm;
  view.mid_cm = g_us_filter.mid.filtered_cm;
  view.right_cm = g_us_filter.right.filtered_cm;
  view.left_valid = g_us_filter.left.valid;
  view.mid_valid = g_us_filter.mid.valid;
  view.right_valid = g_us_filter.right.valid;
  return view;
}

/**
 * @brief Đọc duty PWM hiện tại của 1 motor (giá trị CCR).
 * @param motor Con trỏ motor trái/phải.
 * @return Giá trị CCR hiện tại, 0 nếu tham số không hợp lệ.
 */
static uint32_t App_ReadPwm(const Motor_t *motor)
{
  if ((motor == nullptr) || (motor->htim == nullptr))
  {
    return 0U;
  }
  return __HAL_TIM_GET_COMPARE(motor->htim, motor->pwm_channel);
}

/**
 * @brief Quay tại chỗ sang trái.
 * @param speed Tốc độ phần trăm [0..100].
 */
static void App_PivotLeft(uint8_t speed)
{
  MotorController_SetDriveMode(&g_motor_controller, MOTOR_DRIVE_MODE_SEARCH);
  MotorController_Set(&g_motor_controller, (int16_t)(-((int16_t)speed)), (int16_t)speed);
}

/**
 * @brief Quay tại chỗ sang phải.
 * @param speed Tốc độ phần trăm [0..100].
 */
static void App_PivotRight(uint8_t speed)
{
  MotorController_SetDriveMode(&g_motor_controller, MOTOR_DRIVE_MODE_SEARCH);
  MotorController_Set(&g_motor_controller, (int16_t)speed, (int16_t)(-((int16_t)speed)));
}

/**
 * @brief Kiểm tra một kênh có valid và nằm trong ngưỡng max hay không.
 * @param valid Cờ hợp lệ của kênh.
 * @param cm Khoảng cách đã lọc (cm).
 * @param max_cm Ngưỡng trên.
 * @return 1 nếu hợp lệ và <= max_cm, ngược lại 0.
 */
static uint8_t App_ChannelInRange(uint8_t valid, uint16_t cm, uint16_t max_cm)
{
  return ((valid != 0U) && (cm <= max_cm)) ? 1U : 0U;
}

static uint8_t App_LineIsBlack(uint8_t line_bit)
{
  return (line_bit == 0U) ? 1U : 0U;
}

static uint8_t App_LineAnyBlack(const LineSensorSnapshot &line)
{
  return (uint8_t)(
    App_LineIsBlack(line.line3) |
    App_LineIsBlack(line.line2) |
    App_LineIsBlack(line.line1) |
    App_LineIsBlack(line.line0));
}

static uint8_t App_LineLeftBlack(const LineSensorSnapshot &line)
{
  return (uint8_t)(App_LineIsBlack(line.line3) | App_LineIsBlack(line.line2));
}

static uint8_t App_LineRightBlack(const LineSensorSnapshot &line)
{
  return (uint8_t)(App_LineIsBlack(line.line1) | App_LineIsBlack(line.line0));
}

/**
 * @brief Cập nhật latch quay trái/phải để tránh rung khi target dao động.
 * @param view Dữ liệu khoảng cách đã lọc của 3 kênh.
 *
 * Latch ON khi lệch vượt TURN_MARGIN_ON_CM.
 * Latch OFF khi lệch về vùng trung tâm TURN_MARGIN_OFF_CM.
 */
static void App_ApplyTurnLatches(const TargetView &view)
{
  int16_t heading_err;
  uint8_t left_track;
  uint8_t right_track;

  left_track = App_ChannelInRange(view.left_valid, view.left_cm, SIDE_TRACK_MAX_CM);
  right_track = App_ChannelInRange(view.right_valid, view.right_cm, SIDE_TRACK_MAX_CM);

  /*
   * heading_err = right - left:
   * >0 => đối thủ lệch trái (quay trái),
   * <0 => đối thủ lệch phải (quay phải).
   */
  if ((left_track != 0U) && (right_track != 0U))
  {
    heading_err = (int16_t)view.right_cm - (int16_t)view.left_cm;

    if (heading_err > (int16_t)TURN_MARGIN_ON_CM)
    {
      g_sm.turn_left_latch = 1U;
      g_sm.turn_right_latch = 0U;
    }
    else if (heading_err < -(int16_t)TURN_MARGIN_ON_CM)
    {
      g_sm.turn_right_latch = 1U;
      g_sm.turn_left_latch = 0U;
    }
    else if ((heading_err <= (int16_t)TURN_MARGIN_OFF_CM) &&
             (heading_err >= -(int16_t)TURN_MARGIN_OFF_CM))
    {
      g_sm.turn_left_latch = 0U;
      g_sm.turn_right_latch = 0U;
    }
  }
  else if (left_track != 0U)
  {
    g_sm.turn_left_latch = 1U;
    g_sm.turn_right_latch = 0U;
  }
  else if (right_track != 0U)
  {
    g_sm.turn_right_latch = 1U;
    g_sm.turn_left_latch = 0U;
  }
}

/**
 * @brief Chọn state ứng viên dựa trên target hiện có.
 * @param view Dữ liệu khoảng cách đã lọc.
 * @param now_ms Tick hiện tại.
 * @return State ứng viên (Search/TurnLeft/TurnRight/Lock).
 *
 * Quy tắc:
 * - Nếu không có mục tiêu tin cậy lâu hơn TARGET_LOST_MS => Search.
 * - Nếu có lệch trái/phải => Turn tương ứng.
 * - Nếu ở giữa và MID valid => Lock.
 */
static RobotState App_SelectCandidate(const TargetView &view, uint32_t now_ms)
{
  uint8_t near_on_any;

  near_on_any = (uint8_t)(
    App_ChannelInRange(view.left_valid, view.left_cm, OPPONENT_DETECT_ON_CM) |
    App_ChannelInRange(view.mid_valid, view.mid_cm, OPPONENT_DETECT_ON_CM) |
    App_ChannelInRange(view.right_valid, view.right_cm, OPPONENT_DETECT_ON_CM));

  if (near_on_any != 0U)
  {
    g_sm.opponent_latch = 1U;
    g_sm.last_target_tick_ms = now_ms;
  }
  else if ((now_ms - g_sm.last_target_tick_ms) > TARGET_LOST_MS)
  {
    g_sm.opponent_latch = 0U;
  }

  /* Không có mục tiêu tin cậy -> chạy pattern quét SEARCH. */
  if (g_sm.opponent_latch == 0U)
  {
    g_sm.turn_left_latch = 0U;
    g_sm.turn_right_latch = 0U;
    return RobotState::Search;
  }

  App_ApplyTurnLatches(view);

  if (g_sm.turn_left_latch != 0U)
  {
    return RobotState::TurnLeft;
  }
  if (g_sm.turn_right_latch != 0U)
  {
    return RobotState::TurnRight;
  }
  return (view.mid_valid != 0U) ? RobotState::Lock : RobotState::Search;
}

/**
 * @brief Ghi nhận chuyển trạng thái ngay lập tức.
 * @param next State đích.
 * @param now_ms Tick hiện tại.
 */
static void App_SetState(RobotState next, uint32_t now_ms)
{
  if (g_sm.state == next)
  {
    return;
  }

  g_sm.prev_state = g_sm.state;
  g_sm.state = next;
  g_sm.state_changed = 1U;
  g_sm.state_tick_ms = now_ms;

  if (next == RobotState::Search)
  {
    g_sm.search_phase = SearchPhase::SoftLeft;
    g_sm.search_phase_tick_ms = now_ms;
  }
}

/**
 * @brief Bắt đầu chuỗi EDGE_ESCAPE khi chạm line đen.
 * @param line Snapshot line sensor (bit 0 đen, 1 trắng theo mapping hiện tại).
 * @param now_ms Tick hiện tại.
 *
 * Hướng quay được bias theo bên phát hiện line đen:
 * - Đen bên trái -> ưu tiên quay phải
 * - Đen bên phải -> ưu tiên quay trái
 * - Cả hai/không rõ -> random
 */
static void App_StartEdgeEscape(const LineSensorSnapshot &line, uint32_t now_ms)
{
  uint8_t left_black = App_LineLeftBlack(line);
  uint8_t right_black = App_LineRightBlack(line);

  /*
   * Phản ứng khi gặp biên:
   * 1) Lùi ngắn để rời mép sàn
   * 2) Quay ngẫu nhiên (ưu tiên theo bên đang thấy line đen)
   */
  g_edge.active = 1U;
  g_edge.phase = EdgePhase::Backward;
  g_edge.phase_tick_ms = now_ms;
  g_edge.turn_duration_ms = App_RandRange(EDGE_TURN_MIN_MS, EDGE_TURN_MAX_MS);

  if ((left_black != 0U) && (right_black == 0U))
  {
    g_edge.turn_dir = +1; /* ưu tiên quay phải */
  }
  else if ((right_black != 0U) && (left_black == 0U))
  {
    g_edge.turn_dir = -1; /* ưu tiên quay trái */
  }
  else
  {
    g_edge.turn_dir = ((App_Rand32() & 1UL) != 0UL) ? +1 : -1;
  }

  App_SetState(RobotState::EdgeEscape, now_ms);
}

/**
 * @brief Kết thúc EDGE_ESCAPE và trả về SEARCH.
 * @param now_ms Tick hiện tại.
 */
static void App_StopEdgeEscape(uint32_t now_ms)
{
  g_edge.active = 0U;
  g_edge.phase = EdgePhase::Idle;
  g_edge.turn_dir = 0;
  g_edge.phase_tick_ms = now_ms;
  g_edge.turn_duration_ms = 0U;
  g_sm.pending_state = RobotState::Search;
  g_sm.pending_count = 0U;
  App_SetState(RobotState::Search, now_ms);
}

/**
 * @brief Chạy từng bước EDGE_ESCAPE theo state con (non-blocking).
 * @param now_ms Tick hiện tại.
 * @return 1 nếu EDGE_ESCAPE vẫn đang chiếm quyền điều khiển motor, 0 nếu không.
 */
static uint8_t App_UpdateEdgeEscape(uint32_t now_ms)
{
  uint32_t elapsed;

  if (g_edge.active == 0U)
  {
    return 0U;
  }

  elapsed = now_ms - g_edge.phase_tick_ms;

  /* Chạy non-blocking theo phase + thời gian HAL_GetTick(). */
  switch (g_edge.phase)
  {
    case EdgePhase::Backward:
      MotorController_SetDriveMode(&g_motor_controller, MOTOR_DRIVE_MODE_SEARCH);
      MotorController_Backward(&g_motor_controller, EDGE_BACKWARD_SPEED);
      if (elapsed >= EDGE_BACKWARD_MS)
      {
        g_edge.phase = EdgePhase::Turn;
        g_edge.phase_tick_ms = now_ms;
      }
      break;

    case EdgePhase::Turn:
      if (g_edge.turn_dir < 0)
      {
        App_PivotLeft(EDGE_TURN_SPEED);
      }
      else
      {
        App_PivotRight(EDGE_TURN_SPEED);
      }
      if (elapsed >= g_edge.turn_duration_ms)
      {
        App_StopEdgeEscape(now_ms);
      }
      break;

    case EdgePhase::Idle:
    default:
      App_StopEdgeEscape(now_ms);
      break;
  }

  return (g_edge.active != 0U) ? 1U : 0U;
}

/**
 * @brief Debounce chuyển state để chống rung.
 * @param candidate State ứng viên từ logic quyết định.
 * @param now_ms Tick hiện tại.
 *
 * Chỉ chuyển state khi:
 * - candidate giữ ổn định đủ STATE_CONFIRM_CYCLES
 * - state hiện tại đã giữ đủ STATE_MIN_HOLD_MS
 */
static void App_UpdateStateWithDebounce(RobotState candidate, uint32_t now_ms)
{
  uint8_t need_hold = ((now_ms - g_sm.state_tick_ms) < STATE_MIN_HOLD_MS) ? 1U : 0U;

  g_sm.state_changed = 0U;

  /* Chống rung khi chuyển trạng thái để tránh dao động do nhiễu mẫu. */
  if (candidate != g_sm.pending_state)
  {
    g_sm.pending_state = candidate;
    g_sm.pending_count = 1U;
    return;
  }
  if (g_sm.pending_count < 255U)
  {
    g_sm.pending_count++;
  }

  if (candidate == g_sm.state)
  {
    return;
  }
  if (g_sm.pending_count < STATE_CONFIRM_CYCLES)
  {
    return;
  }
  if (need_hold != 0U)
  {
    return;
  }

  App_SetState(candidate, now_ms);
}

/**
 * @brief Mẫu quét mục tiêu khi không thấy đối thủ.
 * @param now_ms Tick hiện tại.
 *
 * Chu kỳ:
 * - SoftLeft  trong SEARCH_SOFT_WINDOW_MS
 * - SoftRight trong SEARCH_SOFT_WINDOW_MS
 * - HardLeft  trong SEARCH_HARD_WINDOW_MS
 */
static void App_ApplySearchPattern(uint32_t now_ms)
{
  uint32_t elapsed = now_ms - g_sm.search_phase_tick_ms;

  /* Lịch quét: trái nhẹ -> phải nhẹ -> trái mạnh -> lặp lại. */
  switch (g_sm.search_phase)
  {
    case SearchPhase::SoftLeft:
      if (elapsed >= SEARCH_SOFT_WINDOW_MS)
      {
        g_sm.search_phase = SearchPhase::SoftRight;
        g_sm.search_phase_tick_ms = now_ms;
      }
      break;
    case SearchPhase::SoftRight:
      if (elapsed >= SEARCH_SOFT_WINDOW_MS)
      {
        g_sm.search_phase = SearchPhase::HardLeft;
        g_sm.search_phase_tick_ms = now_ms;
      }
      break;
    case SearchPhase::HardLeft:
    default:
      if (elapsed >= SEARCH_HARD_WINDOW_MS)
      {
        g_sm.search_phase = SearchPhase::SoftLeft;
        g_sm.search_phase_tick_ms = now_ms;
      }
      break;
  }

  switch (g_sm.search_phase)
  {
    case SearchPhase::SoftLeft:
      App_PivotLeft(SPD_SEARCH_SOFT);
      break;
    case SearchPhase::SoftRight:
      App_PivotRight(SPD_SEARCH_SOFT);
      break;
    case SearchPhase::HardLeft:
    default:
      App_PivotLeft(SPD_SEARCH_HARD);
      break;
  }
}

/**
 * @brief Áp lệnh motor tương ứng state hiện tại.
 * @param view Dữ liệu mục tiêu đã lọc (hiện chỉ dùng gián tiếp cho logic mở rộng).
 * @param now_ms Tick hiện tại (dùng cho search pattern).
 */
static void App_ApplyStateAction(const TargetView &view, uint32_t now_ms)
{
  switch (g_sm.state)
  {
    case RobotState::Search:
      App_ApplySearchPattern(now_ms);
      break;
    case RobotState::TurnLeft:
      App_PivotLeft(SPD_TURN);
      break;
    case RobotState::TurnRight:
      App_PivotRight(SPD_TURN);
      break;
    case RobotState::Lock:
    default:
      MotorController_SetDriveMode(&g_motor_controller, MOTOR_DRIVE_MODE_SEARCH);
      MotorController_Stop(&g_motor_controller);
      break;
  }
}

/**
 * @brief Vòng state machine chính của ứng dụng.
 * @param snapshot Snapshot sensor tại tick hiện tại.
 * @param now_ms Tick hiện tại.
 *
 * Thứ tự ưu tiên:
 * 1) Boundary/line đen -> EDGE_ESCAPE (ưu tiên cao nhất)
 * 2) Nếu không escape, xử lý ultrasonic -> chọn candidate -> debounce -> áp motor
 */
static void App_RunStateMachine(const SensorManagerSnapshot &snapshot, uint32_t now_ms)
{
  TargetView view;
  RobotState candidate;
  uint8_t line_black;

  /* Xử lý biên có ưu tiên cao nhất, cao hơn bám đối thủ. */
  line_black = App_LineAnyBlack(snapshot.line);

  if ((g_edge.active == 0U) && (line_black != 0U))
  {
    App_StartEdgeEscape(snapshot.line, now_ms);
  }

  if (App_UpdateEdgeEscape(now_ms) != 0U)
  {
    return;
  }

  App_UpdateUsFilter(snapshot, now_ms);
  view = App_BuildTargetView();
  candidate = App_SelectCandidate(view, now_ms);
  App_UpdateStateWithDebounce(candidate, now_ms);
  App_ApplyStateAction(view, now_ms);
}

/**
 * @brief Log trạng thái runtime gói gọn 1 dòng để debug qua UART.
 * @param snapshot Snapshot sensor gần nhất.
 *
 * Trường log gồm:
 * - State tổng quát + edge/search phase
 * - Ultrasonic filtered/raw + valid bits
 * - Motor target/applied/pwm/fault
 * - Line bits + has_line
 */
static void App_LogStateRateLimited(const SensorManagerSnapshot &snapshot)
{
  TargetView view = App_BuildTargetView();
  uint32_t edge_left_ms = 0U;

  if (g_edge.active != 0U)
  {
    uint32_t elapsed = HAL_GetTick() - g_edge.phase_tick_ms;
    uint32_t phase_total = 0U;

    if (g_edge.phase == EdgePhase::Backward)
    {
      phase_total = EDGE_BACKWARD_MS;
    }
    else if (g_edge.phase == EdgePhase::Turn)
    {
      phase_total = g_edge.turn_duration_ms;
    }

    edge_left_ms = (elapsed < phase_total) ? (phase_total - elapsed) : 0U;
  }

  /*
   * Log debug 1 dòng gọn:
   * state/chg/opp/search-phase/edge-phase + ultrasonic lọc/raw + lệnh motor + bit line.
   */
  UartLogger::instance().logRateLimited(
    LogLevel::Info,
    "STATE",
    KEY_STATE,
    APP_STATE_LOG_MS,
    "sm=%s chg=%u opp=%u sp=%s edge=%u/%s/%ld/%d us=%u,%u,%u v=%u%u%u raw=%u,%u,%u mot=%d/%d|%d/%d|%lu/%lu|0x%02lX line=%u%u%u%u h=%u",
    App_StateName(g_sm.state),
    (unsigned int)g_sm.state_changed,
    (unsigned int)g_sm.opponent_latch,
    App_SearchPhaseName(g_sm.search_phase),
    (unsigned int)g_edge.active,
    App_EdgePhaseName(g_edge.phase),
    (long)edge_left_ms,
    (int)g_edge.turn_dir,
    (unsigned int)view.left_cm,
    (unsigned int)view.left_valid,
    (unsigned int)view.mid_cm,
    (unsigned int)view.mid_valid,
    (unsigned int)view.right_cm,
    (unsigned int)view.right_valid,
    (unsigned int)snapshot.ultrasonic.left.raw_distance_cm,
    (unsigned int)snapshot.ultrasonic.mid.raw_distance_cm,
    (unsigned int)snapshot.ultrasonic.right.raw_distance_cm,
    (int)g_motor_controller.target_left_percent,
    (int)g_motor_controller.target_right_percent,
    (int)g_motor_controller.applied_left_percent,
    (int)g_motor_controller.applied_right_percent,
    (unsigned long)App_ReadPwm(&g_motor_controller.left),
    (unsigned long)App_ReadPwm(&g_motor_controller.right),
    (unsigned long)MotorController_GetFaults(&g_motor_controller),
    (unsigned int)snapshot.line.line3,
    (unsigned int)snapshot.line.line2,
    (unsigned int)snapshot.line.line1,
    (unsigned int)snapshot.line.line0,
    (unsigned int)snapshot.line.has_line);
}

/**
 * @brief Cập nhật LED theo state hiện tại của robot.
 *
 * Quy ước hiển thị:
 * - SEARCH      : chớp rất chậm
 * - TURN_LEFT   : chớp rất nhanh
 * - TURN_RIGHT  : chớp trung bình
 * - EDGE_ESCAPE : chớp cực nhanh
 * - LOCK        : sáng liên tục
 *
 * Lưu ý: chỉ gọi LedManager_SetMode khi mode thực sự thay đổi để không reset chu kỳ chớp.
 */
static void App_UpdateLedByState(void)
{
  LedMode_t desired_mode;
  uint32_t custom_period_ms = 0U;

  switch (g_sm.state)
  {
    case RobotState::Search:
      desired_mode = LED_MODE_BLINK_CUSTOM;
      custom_period_ms = LED_SEARCH_PERIOD_MS;
      break;
    case RobotState::TurnLeft:
      desired_mode = LED_MODE_BLINK_CUSTOM;
      custom_period_ms = LED_TURN_LEFT_PERIOD_MS;
      break;
    case RobotState::TurnRight:
      desired_mode = LED_MODE_BLINK_CUSTOM;
      custom_period_ms = LED_TURN_RIGHT_PERIOD_MS;
      break;
    case RobotState::EdgeEscape:
      desired_mode = LED_MODE_BLINK_CUSTOM;
      custom_period_ms = LED_EDGE_PERIOD_MS;
      break;
    case RobotState::Lock:
    default:
      desired_mode = LED_MODE_ON;
      break;
  }

  if (desired_mode == LED_MODE_BLINK_CUSTOM)
  {
    LedManager_SetCustomBlink(custom_period_ms);
  }

  if (LedManager_GetMode() != desired_mode)
  {
    LedManager_SetMode(desired_mode);
  }
}
}

/**
 * @brief Khởi tạo ứng dụng (UART log, sensor manager, motor, state context).
 */
extern "C" void AppMain_Init(void)
{
  UartLogger &logger = UartLogger::instance();
  uint32_t now_ms;

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
  MotorController_SetInversion(&g_motor_controller, MOTOR_INVERT_LEFT, MOTOR_INVERT_RIGHT);
  if (MotorController_Start(&g_motor_controller) != HAL_OK)
  {
    LOGW("INIT", "MotorController start failed");
  }

  now_ms = HAL_GetTick();
  g_heartbeat_tick = now_ms;
  g_drop_tick = now_ms;
  g_last_dropped = 0U;

  g_us_filter = {};
  g_edge.active = 0U;
  g_edge.phase = EdgePhase::Idle;
  g_edge.turn_dir = 0;
  g_edge.phase_tick_ms = now_ms;
  g_edge.turn_duration_ms = 0U;
  g_rand_state ^= (now_ms + 0x9E3779B9UL);
  g_sm.state = RobotState::Search;
  g_sm.prev_state = RobotState::Search;
  g_sm.pending_state = RobotState::Search;
  g_sm.state_changed = 0U;
  g_sm.pending_count = 0U;
  g_sm.turn_left_latch = 0U;
  g_sm.turn_right_latch = 0U;
  g_sm.opponent_latch = 0U;
  g_sm.state_tick_ms = now_ms;
  g_sm.last_target_tick_ms = now_ms;
  g_sm.search_phase_tick_ms = now_ms;
  g_sm.search_phase = SearchPhase::SoftLeft;

  LOGI(
    "INIT",
    "SM rotate-only ready: turn=%u/%u detect<=%u search(soft/hard)=%lu/%lums edge(back/turn)=%lu/%lu-%lums",
    (unsigned int)TURN_MARGIN_ON_CM,
    (unsigned int)TURN_MARGIN_OFF_CM,
    (unsigned int)OPPONENT_DETECT_ON_CM,
    (unsigned long)SEARCH_SOFT_WINDOW_MS,
    (unsigned long)SEARCH_HARD_WINDOW_MS,
    (unsigned long)EDGE_BACKWARD_MS,
    (unsigned long)EDGE_TURN_MIN_MS,
    (unsigned long)EDGE_TURN_MAX_MS);
}

/**
 * @brief Vòng lặp ứng dụng gọi liên tục từ main (non-blocking).
 *
 * Chu trình mỗi vòng:
 * 1) update sensor manager
 * 2) chạy state machine
 * 3) update motor + led
 * 4) log rate-limited
 * 5) flush logger
 */
extern "C" void AppMain_Loop(void)
{
  uint32_t now_ms = HAL_GetTick();
  SensorEventFlags events = g_sensor_manager.update(now_ms);
  const SensorManagerSnapshot &snapshot = g_sensor_manager.snapshot();
  UartLogger &logger = UartLogger::instance();

  App_RunStateMachine(snapshot, now_ms);
  MotorController_Update(&g_motor_controller);

  App_UpdateLedByState();
  LedManager_Update();

  if (((events & SensorManager::EVENT_LINE_UPDATED) != 0U) ||
      ((events & SensorManager::EVENT_ULTRASONIC_UPDATED) != 0U))
  {
    App_LogStateRateLimited(snapshot);
  }

  if ((APP_HEARTBEAT_MS > 0U) && ((now_ms - g_heartbeat_tick) >= APP_HEARTBEAT_MS))
  {
    g_heartbeat_tick = now_ms;
    LOGI("HB", "sm=%s", App_StateName(g_sm.state));
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
