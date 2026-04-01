#ifndef ISENSOR_HPP
#define ISENSOR_HPP

#include "modules/sensors/sensor_types.hpp"
#include <stdint.h>

/* Interface chung để app có thể thao tác cảm biến theo một khuôn duy nhất:
   init -> start -> update -> đọc snapshot. */
class ISensor
{
public:
  virtual ~ISensor() {}

  virtual void init() = 0;
  virtual bool start() = 0;
  virtual void stop() = 0;
  virtual SensorEventFlags update(uint32_t now_ms) = 0;
  virtual const SensorSnapshot &snapshot() const = 0;
  virtual const char *name() const = 0;
};

#endif /* ISENSOR_HPP */

