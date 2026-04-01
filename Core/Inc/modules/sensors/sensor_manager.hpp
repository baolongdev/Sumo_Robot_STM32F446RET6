#ifndef SENSOR_MANAGER_HPP
#define SENSOR_MANAGER_HPP

#include "modules/sensors/line_sensor_class.hpp"
#include "modules/sensors/ultrasonic_array.hpp"

struct SensorManagerConfig
{
  /* Sensor manager chỉ là lớp ghép 2 loại sensor vào một cấu hình chung. */
  LineSensorConfig line;
  UltrasonicArrayConfig ultrasonic;
};

struct SensorManagerSnapshot
{
  SensorTickMs tick_ms;
  SensorStatus status;
  uint8_t valid;
  SensorEventFlags event_flags;
  LineSensorSnapshot line;
  UltrasonicArraySnapshot ultrasonic;
};

class SensorManager
{
public:
  /* Opponent mask cho phép app biết đối thủ đang nằm ở trái/giữa/phải. */
  enum : uint8_t
  {
    OPPONENT_MASK_NONE = 0U,
    OPPONENT_MASK_LEFT = (1U << 0),
    OPPONENT_MASK_MID = (1U << 1),
    OPPONENT_MASK_RIGHT = (1U << 2)
  };

  /* Event tổng hợp của manager ở mức "nguồn nào vừa cập nhật". */
  enum : SensorEventFlags
  {
    EVENT_NONE = SensorEventNone,
    EVENT_LINE_UPDATED = SensorEventBit(SensorEvent::LineUpdated),
    EVENT_ULTRASONIC_UPDATED = SensorEventBit(SensorEvent::UltrasonicUpdated)
  };

  explicit SensorManager(const SensorManagerConfig &config);

  void setConfig(const SensorManagerConfig &config);
  void init();
  bool start();
  void stop();
  SensorEventFlags update(uint32_t now_ms);

  const SensorManagerSnapshot &snapshot() const;
  bool isBoundaryDetected() const;
  uint8_t opponentMaskWithinCm(uint16_t threshold_cm) const;
  bool hasOpponentWithinCm(uint16_t threshold_cm) const;
  uint16_t nearestOpponentCm() const;
  uint8_t nearestOpponentMask() const;
  bool isDataFresh(uint32_t now_ms, uint32_t stale_ms) const;

  const LineSensor &line() const;
  const UltrasonicArray &ultrasonic() const;

private:
  SensorManagerConfig config_;
  LineSensor line_sensor_;
  UltrasonicArray ultrasonic_array_;
  SensorManagerSnapshot snapshot_;
};

#endif /* SENSOR_MANAGER_HPP */

