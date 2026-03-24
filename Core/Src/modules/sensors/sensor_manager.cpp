#include "modules/sensors/sensor_manager.hpp"

namespace
{
constexpr uint16_t kDistanceInvalidCm = 0xFFFFU;

static uint16_t MinValidCm(uint16_t current_min, const UltrasonicChannelSnapshot &channel)
{
  if ((channel.valid == 0U) || (channel.status != SensorStatus::Ok))
  {
    return current_min;
  }
  if (channel.distance_cm < current_min)
  {
    return channel.distance_cm;
  }
  return current_min;
}
}

SensorManager::SensorManager(const SensorManagerConfig &config)
  : config_(config),
    line_sensor_(config.line),
    ultrasonic_array_(config.ultrasonic)
{
  snapshot_.tick_ms = 0U;
  snapshot_.status = SensorStatus::Idle;
  snapshot_.valid = 0U;
  snapshot_.event_flags = EVENT_NONE;
}

void SensorManager::setConfig(const SensorManagerConfig &config)
{
  config_ = config;
  line_sensor_.setConfig(config.line);
  ultrasonic_array_.setConfig(config.ultrasonic);
}

void SensorManager::init()
{
  line_sensor_.init();
  ultrasonic_array_.init();

  snapshot_.tick_ms = HAL_GetTick();
  snapshot_.status = SensorStatus::Idle;
  snapshot_.valid = 0U;
  snapshot_.event_flags = EVENT_NONE;
  snapshot_.line = line_sensor_.lineSnapshot();
  snapshot_.ultrasonic = ultrasonic_array_.ultrasonicSnapshot();
}

bool SensorManager::start()
{
  bool line_ok = line_sensor_.start();
  bool us_ok = ultrasonic_array_.start();
  return (line_ok && us_ok);
}

void SensorManager::stop()
{
  uint32_t now_ms = HAL_GetTick();

  line_sensor_.stop();
  ultrasonic_array_.stop();

  snapshot_.tick_ms = now_ms;
  snapshot_.status = SensorStatus::Idle;
  snapshot_.valid = 0U;
  snapshot_.event_flags = EVENT_NONE;
  snapshot_.line = line_sensor_.lineSnapshot();
  snapshot_.ultrasonic = ultrasonic_array_.ultrasonicSnapshot();
}

SensorEventFlags SensorManager::update(uint32_t now_ms)
{
  SensorEventFlags event_flags = EVENT_NONE;
  SensorEventFlags line_events = line_sensor_.update(now_ms);
  SensorEventFlags us_events = ultrasonic_array_.update(now_ms);
  SensorStatus overall_status;
  uint8_t overall_valid;

  if (line_events != SensorEventNone)
  {
    SensorEventSet(event_flags, SensorEvent::LineUpdated);
    event_flags |= line_events;
  }

  if (us_events != SensorEventNone)
  {
    SensorEventSet(event_flags, SensorEvent::UltrasonicUpdated);
    event_flags |= us_events;
  }

  if (event_flags != EVENT_NONE)
  {
    snapshot_.tick_ms = now_ms;
    snapshot_.event_flags = event_flags;
    snapshot_.line = line_sensor_.lineSnapshot();
    snapshot_.ultrasonic = ultrasonic_array_.ultrasonicSnapshot();

    overall_status = SensorStatus::Idle;
    if ((snapshot_.line.status == SensorStatus::Error) || (snapshot_.ultrasonic.status == SensorStatus::Error))
    {
      overall_status = SensorStatus::Error;
    }
    else if ((snapshot_.line.status == SensorStatus::Timeout) || (snapshot_.ultrasonic.status == SensorStatus::Timeout))
    {
      overall_status = SensorStatus::Timeout;
    }
    else if ((snapshot_.line.status == SensorStatus::Busy) || (snapshot_.ultrasonic.status == SensorStatus::Busy))
    {
      overall_status = SensorStatus::Busy;
    }
    else if ((snapshot_.line.status == SensorStatus::Ok) || (snapshot_.ultrasonic.status == SensorStatus::Ok))
    {
      overall_status = SensorStatus::Ok;
    }

    overall_valid = ((snapshot_.line.valid != 0U) || (snapshot_.ultrasonic.valid != 0U)) ? 1U : 0U;
    snapshot_.status = overall_status;
    snapshot_.valid = overall_valid;
  }

  return event_flags;
}

const SensorManagerSnapshot &SensorManager::snapshot() const
{
  return snapshot_;
}

bool SensorManager::isBoundaryDetected() const
{
  return (snapshot_.line.valid != 0U) && (snapshot_.line.has_line != 0U);
}

uint8_t SensorManager::opponentMaskWithinCm(uint16_t threshold_cm) const
{
  uint8_t mask = OPPONENT_MASK_NONE;

  if ((snapshot_.ultrasonic.left.valid != 0U) &&
      (snapshot_.ultrasonic.left.status == SensorStatus::Ok) &&
      (snapshot_.ultrasonic.left.distance_cm <= threshold_cm))
  {
    mask |= OPPONENT_MASK_LEFT;
  }
  if ((snapshot_.ultrasonic.mid.valid != 0U) &&
      (snapshot_.ultrasonic.mid.status == SensorStatus::Ok) &&
      (snapshot_.ultrasonic.mid.distance_cm <= threshold_cm))
  {
    mask |= OPPONENT_MASK_MID;
  }
  if ((snapshot_.ultrasonic.right.valid != 0U) &&
      (snapshot_.ultrasonic.right.status == SensorStatus::Ok) &&
      (snapshot_.ultrasonic.right.distance_cm <= threshold_cm))
  {
    mask |= OPPONENT_MASK_RIGHT;
  }

  return mask;
}

bool SensorManager::hasOpponentWithinCm(uint16_t threshold_cm) const
{
  return opponentMaskWithinCm(threshold_cm) != OPPONENT_MASK_NONE;
}

uint16_t SensorManager::nearestOpponentCm() const
{
  uint16_t min_cm = kDistanceInvalidCm;

  min_cm = MinValidCm(min_cm, snapshot_.ultrasonic.left);
  min_cm = MinValidCm(min_cm, snapshot_.ultrasonic.mid);
  min_cm = MinValidCm(min_cm, snapshot_.ultrasonic.right);

  return min_cm;
}

uint8_t SensorManager::nearestOpponentMask() const
{
  uint16_t nearest_cm = nearestOpponentCm();
  uint8_t mask = OPPONENT_MASK_NONE;

  if (nearest_cm == kDistanceInvalidCm)
  {
    return mask;
  }

  if ((snapshot_.ultrasonic.left.valid != 0U) &&
      (snapshot_.ultrasonic.left.status == SensorStatus::Ok) &&
      (snapshot_.ultrasonic.left.distance_cm == nearest_cm))
  {
    mask |= OPPONENT_MASK_LEFT;
  }
  if ((snapshot_.ultrasonic.mid.valid != 0U) &&
      (snapshot_.ultrasonic.mid.status == SensorStatus::Ok) &&
      (snapshot_.ultrasonic.mid.distance_cm == nearest_cm))
  {
    mask |= OPPONENT_MASK_MID;
  }
  if ((snapshot_.ultrasonic.right.valid != 0U) &&
      (snapshot_.ultrasonic.right.status == SensorStatus::Ok) &&
      (snapshot_.ultrasonic.right.distance_cm == nearest_cm))
  {
    mask |= OPPONENT_MASK_RIGHT;
  }

  return mask;
}

bool SensorManager::isDataFresh(uint32_t now_ms, uint32_t stale_ms) const
{
  return (now_ms - snapshot_.tick_ms) <= stale_ms;
}

const LineSensor &SensorManager::line() const
{
  return line_sensor_;
}

const UltrasonicArray &SensorManager::ultrasonic() const
{
  return ultrasonic_array_;
}

