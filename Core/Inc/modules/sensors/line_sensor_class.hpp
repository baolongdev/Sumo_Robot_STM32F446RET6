#ifndef LINE_SENSOR_CLASS_HPP
#define LINE_SENSOR_CLASS_HPP

#include "i2c.h"
#include "modules/sensors/isensor.hpp"
#include "modules/sensors/line_sensor.h"

struct LineSensorConfig
{
  I2C_HandleTypeDef *hi2c;
  uint8_t address_7bit;
  uint16_t timeout_ms;
  uint32_t poll_interval_ms;
  uint8_t active_low;
};

class LineSensor : public ISensor
{
public:
  explicit LineSensor(const LineSensorConfig &config);

  void setConfig(const LineSensorConfig &config);
  void init() override;
  bool start() override;
  void stop() override;
  SensorEventFlags update(uint32_t now_ms) override;
  const SensorSnapshot &snapshot() const override;
  const LineSensorSnapshot &lineSnapshot() const;
  const char *name() const override;

private:
  LineSensorConfig config_;
  LineSensor_t driver_;
  LineSensorSnapshot snapshot_;
};

#endif /* LINE_SENSOR_CLASS_HPP */

