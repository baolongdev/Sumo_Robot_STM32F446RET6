#ifndef ULTRASONIC_ARRAY_HPP
#define ULTRASONIC_ARRAY_HPP

#include "modules/sensors/isensor.hpp"
#include "tim.h"
#include "modules/sensors/ultrasonic_manager.h"

struct UltrasonicArrayConfig
{
  /* Các tham số timing ở đây tác động trực tiếp đến dải đo và tốc độ làm mới. */
  TIM_HandleTypeDef *htim_us;
  uint16_t trigger_pulse_us;
  uint16_t echo_rise_timeout_us;
  uint16_t echo_fall_timeout_us;
  uint16_t inter_sensor_gap_us;
  uint8_t kalman_enabled;
  float kalman_q;
  float kalman_r;
};

class UltrasonicArray : public ISensor
{
public:
  explicit UltrasonicArray(const UltrasonicArrayConfig &config);

  void setConfig(const UltrasonicArrayConfig &config);
  void init() override;
  bool start() override;
  void stop() override;
  SensorEventFlags update(uint32_t now_ms) override;
  const SensorSnapshot &snapshot() const override;
  const UltrasonicArraySnapshot &ultrasonicSnapshot() const;
  const char *name() const override;

private:
  /* FillChannel chuyển dữ liệu từ driver C sang snapshot C++ thống nhất cho app. */
  void FillChannel(UltrasonicChannelSnapshot *dst, UltrasonicSensorId_t id) const;

  UltrasonicArrayConfig config_;
  UltrasonicManager_t driver_;
  UltrasonicArraySnapshot snapshot_;
};

#endif /* ULTRASONIC_ARRAY_HPP */

