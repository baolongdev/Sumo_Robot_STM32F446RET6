#include "modules/sensors/ultrasonic_array.hpp"

UltrasonicArray::UltrasonicArray(const UltrasonicArrayConfig &config)
  : config_(config)
{
  snapshot_.tick_ms = 0U;
  snapshot_.status = SensorStatus::Idle;
  snapshot_.valid = 0U;
  snapshot_.event_flags = SensorEventNone;
  snapshot_.left.tick_ms = 0U;
  snapshot_.left.raw_distance_cm = 0U;
  snapshot_.left.distance_cm = 0U;
  snapshot_.left.pulse_us = 0U;
  snapshot_.left.status = SensorStatus::Idle;
  snapshot_.left.valid = 0U;
  snapshot_.mid = snapshot_.left;
  snapshot_.right = snapshot_.left;
}

void UltrasonicArray::setConfig(const UltrasonicArrayConfig &config)
{
  config_ = config;
}

void UltrasonicArray::init()
{
  UltrasonicManager_InitDefault(&driver_, config_.htim_us);
  driver_.config.trigger_pulse_us = config_.trigger_pulse_us;
  driver_.config.echo_rise_timeout_us = config_.echo_rise_timeout_us;
  driver_.config.echo_fall_timeout_us = config_.echo_fall_timeout_us;
  driver_.config.inter_sensor_gap_us = config_.inter_sensor_gap_us;
  UltrasonicManager_SetKalman(&driver_, config_.kalman_enabled, config_.kalman_q, config_.kalman_r);

  snapshot_.tick_ms = HAL_GetTick();
  snapshot_.status = SensorStatus::Idle;
  snapshot_.valid = 0U;
  snapshot_.event_flags = SensorEventNone;
}

bool UltrasonicArray::start()
{
  return (UltrasonicManager_Start(&driver_) == HAL_OK);
}

void UltrasonicArray::stop()
{
  UltrasonicManager_Stop(&driver_);
  snapshot_.tick_ms = HAL_GetTick();
  snapshot_.status = SensorStatus::Idle;
  snapshot_.valid = 0U;
  snapshot_.event_flags = SensorEventNone;
}

SensorEventFlags UltrasonicArray::update(uint32_t now_ms)
{
  uint8_t update_mask = UltrasonicManager_Update(&driver_, now_ms);
  SensorEventFlags event_flags = SensorEventNone;
  SensorStatus overall_status = SensorStatus::Idle;
  UltrasonicArraySnapshot prev_snapshot = snapshot_;

  if (update_mask == 0U)
  {
    return SensorEventNone;
  }

  FillChannel(&snapshot_.left, ULTRASONIC_SENSOR_LEFT);
  FillChannel(&snapshot_.mid, ULTRASONIC_SENSOR_MID);
  FillChannel(&snapshot_.right, ULTRASONIC_SENSOR_RIGHT);

  if ((snapshot_.left.valid != 0U) || (snapshot_.mid.valid != 0U) || (snapshot_.right.valid != 0U))
  {
    overall_status = SensorStatus::Ok;
  }

  if ((snapshot_.left.status == SensorStatus::Timeout) ||
      (snapshot_.mid.status == SensorStatus::Timeout) ||
      (snapshot_.right.status == SensorStatus::Timeout))
  {
    overall_status = SensorStatus::Timeout;
  }
  else if ((snapshot_.left.status == SensorStatus::Error) ||
           (snapshot_.mid.status == SensorStatus::Error) ||
           (snapshot_.right.status == SensorStatus::Error))
  {
    overall_status = SensorStatus::Error;
  }

  SensorEventSet(event_flags, SensorEvent::Sample);

  if (snapshot_.status != overall_status)
  {
    SensorEventSet(event_flags, SensorEvent::StatusChanged);
  }

  if ((snapshot_.left.distance_cm != prev_snapshot.left.distance_cm) ||
      (snapshot_.mid.distance_cm != prev_snapshot.mid.distance_cm) ||
      (snapshot_.right.distance_cm != prev_snapshot.right.distance_cm) ||
      (snapshot_.left.status != prev_snapshot.left.status) ||
      (snapshot_.mid.status != prev_snapshot.mid.status) ||
      (snapshot_.right.status != prev_snapshot.right.status))
  {
    SensorEventSet(event_flags, SensorEvent::StateChanged);
  }

  snapshot_.tick_ms = now_ms;
  snapshot_.status = overall_status;
  snapshot_.valid = ((snapshot_.left.valid != 0U) || (snapshot_.mid.valid != 0U) || (snapshot_.right.valid != 0U)) ? 1U : 0U;
  snapshot_.event_flags = event_flags;

  return event_flags;
}

const SensorSnapshot &UltrasonicArray::snapshot() const
{
  return snapshot_;
}

const UltrasonicArraySnapshot &UltrasonicArray::ultrasonicSnapshot() const
{
  return snapshot_;
}

const char *UltrasonicArray::name() const
{
  return "ULTRASONIC";
}

void UltrasonicArray::FillChannel(UltrasonicChannelSnapshot *dst, UltrasonicSensorId_t id) const
{
  const UltrasonicReading_t *src;

  if (dst == NULL)
  {
    return;
  }

  src = UltrasonicManager_GetReading(&driver_, id);
  if (src == NULL)
  {
    dst->tick_ms = HAL_GetTick();
    dst->raw_distance_cm = 0U;
    dst->distance_cm = 0U;
    dst->pulse_us = 0U;
    dst->status = SensorStatus::Error;
    dst->valid = 0U;
    return;
  }

  dst->tick_ms = src->tick_ms;
  dst->raw_distance_cm = src->raw_distance_cm;
  dst->distance_cm = src->distance_cm;
  dst->pulse_us = src->pulse_us;
  dst->status = SensorStatusFromUltrasonic(src->status);
  dst->valid = src->valid;
}

