#ifndef SENSOR_TYPES_HPP
#define SENSOR_TYPES_HPP

#include "main.h"
#include "modules/sensors/line_sensor.h"
#include "modules/sensors/ultrasonic_manager.h"
#include <stdint.h>

using SensorTickMs = uint32_t;
using SensorEventFlags = uint32_t;

/* Unified runtime status across all sensor modules. */
enum class SensorStatus : uint8_t
{
  Idle = 0,
  Ok,
  Timeout,
  Error,
  Busy
};

/* Unified event bit convention for all sensor snapshots/managers. */
enum class SensorEvent : SensorEventFlags
{
  None = 0U,
  Sample = (1UL << 0),
  StatusChanged = (1UL << 1),
  StateChanged = (1UL << 2),
  Enter = (1UL << 3),
  Exit = (1UL << 4),
  LineUpdated = (1UL << 16),
  UltrasonicUpdated = (1UL << 17)
};

constexpr SensorEventFlags SensorEventBit(SensorEvent event)
{
  return static_cast<SensorEventFlags>(event);
}

constexpr SensorEventFlags SensorEventNone = SensorEventBit(SensorEvent::None);

constexpr bool SensorEventHas(SensorEventFlags event_flags, SensorEvent event)
{
  return (event_flags & SensorEventBit(event)) != 0U;
}

constexpr void SensorEventSet(SensorEventFlags &event_flags, SensorEvent event)
{
  event_flags |= SensorEventBit(event);
}

/* Legacy aliases for compatibility with existing app code. */
constexpr SensorEventFlags SENSOR_EVENT_NONE = SensorEventBit(SensorEvent::None);
constexpr SensorEventFlags SENSOR_EVENT_SAMPLE = SensorEventBit(SensorEvent::Sample);
constexpr SensorEventFlags SENSOR_EVENT_STATUS_CHANGED = SensorEventBit(SensorEvent::StatusChanged);
constexpr SensorEventFlags SENSOR_EVENT_STATE_CHANGED = SensorEventBit(SensorEvent::StateChanged);
constexpr SensorEventFlags SENSOR_EVENT_ENTER = SensorEventBit(SensorEvent::Enter);
constexpr SensorEventFlags SENSOR_EVENT_EXIT = SensorEventBit(SensorEvent::Exit);

static inline SensorStatus SensorStatusFromHal(HAL_StatusTypeDef status)
{
  switch (status)
  {
    case HAL_OK:
      return SensorStatus::Ok;
    case HAL_TIMEOUT:
      return SensorStatus::Timeout;
    case HAL_BUSY:
      return SensorStatus::Busy;
    case HAL_ERROR:
    default:
      return SensorStatus::Error;
  }
}

static inline SensorStatus SensorStatusFromUltrasonic(UltrasonicStatus_t status)
{
  switch (status)
  {
    case ULTRASONIC_STATUS_OK:
      return SensorStatus::Ok;
    case ULTRASONIC_STATUS_TIMEOUT_RISE:
    case ULTRASONIC_STATUS_TIMEOUT_FALL:
      return SensorStatus::Timeout;
    case ULTRASONIC_STATUS_IDLE:
      return SensorStatus::Idle;
    default:
      return SensorStatus::Error;
  }
}

static inline SensorEventFlags SensorEventFromLineEventFlags(uint8_t line_event_flags)
{
  SensorEventFlags event_flags = SensorEventNone;

  if ((line_event_flags & LINE_SENSOR_EVENT_SAMPLE) != 0U)
  {
    SensorEventSet(event_flags, SensorEvent::Sample);
  }
  if ((line_event_flags & LINE_SENSOR_EVENT_STATUS_CHANGED) != 0U)
  {
    SensorEventSet(event_flags, SensorEvent::StatusChanged);
  }
  if ((line_event_flags & LINE_SENSOR_EVENT_STATE_CHANGED) != 0U)
  {
    SensorEventSet(event_flags, SensorEvent::StateChanged);
  }
  if ((line_event_flags & LINE_SENSOR_EVENT_LINE_ENTER) != 0U)
  {
    SensorEventSet(event_flags, SensorEvent::Enter);
  }
  if ((line_event_flags & LINE_SENSOR_EVENT_LINE_EXIT) != 0U)
  {
    SensorEventSet(event_flags, SensorEvent::Exit);
  }

  return event_flags;
}

struct SensorSnapshot
{
  SensorTickMs tick_ms;
  SensorStatus status;
  uint8_t valid;
  SensorEventFlags event_flags;
};

/* Snapshot for 4-bit line sensor state. */
struct LineSensorSnapshot : public SensorSnapshot
{
  uint8_t raw_data;
  uint8_t sensor_nibble;
  uint8_t has_line;
  uint8_t line3;
  uint8_t line2;
  uint8_t line1;
  uint8_t line0;
};

/* Snapshot for one ultrasonic channel. */
struct UltrasonicChannelSnapshot
{
  SensorTickMs tick_ms;
  uint16_t raw_distance_cm;
  uint16_t distance_cm;
  uint16_t pulse_us;
  SensorStatus status;
  uint8_t valid;
};

/* Snapshot for 3-channel ultrasonic array. */
struct UltrasonicArraySnapshot : public SensorSnapshot
{
  UltrasonicChannelSnapshot left;
  UltrasonicChannelSnapshot mid;
  UltrasonicChannelSnapshot right;
};

static inline const char *SensorStatusName(SensorStatus status)
{
  switch (status)
  {
    case SensorStatus::Idle:
      return "IDLE";
    case SensorStatus::Ok:
      return "OK";
    case SensorStatus::Timeout:
      return "TIMEOUT";
    case SensorStatus::Error:
      return "ERROR";
    case SensorStatus::Busy:
      return "BUSY";
    default:
      return "UNKNOWN";
  }
}

#endif /* SENSOR_TYPES_HPP */
