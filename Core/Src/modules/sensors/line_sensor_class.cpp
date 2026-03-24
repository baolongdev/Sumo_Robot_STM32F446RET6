#include "modules/sensors/line_sensor_class.hpp"

LineSensor::LineSensor(const LineSensorConfig &config)
  : config_(config)
{
  snapshot_.tick_ms = 0U;
  snapshot_.status = SensorStatus::Idle;
  snapshot_.valid = 0U;
  snapshot_.event_flags = SensorEventNone;
  snapshot_.raw_data = 0U;
  snapshot_.sensor_nibble = 0U;
  snapshot_.has_line = 0U;
  snapshot_.line3 = 0U;
  snapshot_.line2 = 0U;
  snapshot_.line1 = 0U;
  snapshot_.line0 = 0U;
}

void LineSensor::setConfig(const LineSensorConfig &config)
{
  config_ = config;
}

void LineSensor::init()
{
  LineSensorPolarity_t polarity = (config_.active_low != 0U) ? LINE_SENSOR_POLARITY_ACTIVE_LOW : LINE_SENSOR_POLARITY_ACTIVE_HIGH;

  LineSensor_Init(&driver_, config_.hi2c, config_.address_7bit);
  LineSensor_SetTimeout(&driver_, config_.timeout_ms);
  LineSensor_SetPollInterval(&driver_, config_.poll_interval_ms);
  LineSensor_SetPolarity(&driver_, polarity);

  snapshot_.tick_ms = HAL_GetTick();
  snapshot_.status = SensorStatus::Idle;
  snapshot_.valid = 0U;
  snapshot_.event_flags = SensorEventNone;
}

bool LineSensor::start()
{
  LineSensor_ResetScheduler(&driver_, HAL_GetTick());
  return true;
}

void LineSensor::stop()
{
  snapshot_.tick_ms = HAL_GetTick();
  snapshot_.status = SensorStatus::Idle;
  snapshot_.valid = 0U;
  snapshot_.event_flags = SensorEventNone;
}

SensorEventFlags LineSensor::update(uint32_t now_ms)
{
  const LineSensorSnapshot_t *src;
  SensorEventFlags event_flags;

  if (LineSensor_Update(&driver_, now_ms) == 0U)
  {
    return SensorEventNone;
  }

  src = LineSensor_GetSnapshot(&driver_);
  if (src == NULL)
  {
    return SensorEventNone;
  }

  event_flags = SensorEventFromLineEventFlags(src->event_flags);

  snapshot_.tick_ms = src->tick_ms;
  snapshot_.status = SensorStatusFromHal(src->status);
  snapshot_.valid = (snapshot_.status == SensorStatus::Ok) ? 1U : 0U;
  snapshot_.event_flags = event_flags;
  snapshot_.raw_data = src->raw_data;
  snapshot_.sensor_nibble = src->sensor_nibble;
  snapshot_.has_line = src->has_line;
  snapshot_.line3 = src->line3;
  snapshot_.line2 = src->line2;
  snapshot_.line1 = src->line1;
  snapshot_.line0 = src->line0;

  return event_flags;
}

const SensorSnapshot &LineSensor::snapshot() const
{
  return snapshot_;
}

const LineSensorSnapshot &LineSensor::lineSnapshot() const
{
  return snapshot_;
}

const char *LineSensor::name() const
{
  return "LINE";
}

