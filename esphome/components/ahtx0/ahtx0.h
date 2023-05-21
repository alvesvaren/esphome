#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"
#include <Adafruit_AHTX0.h>

namespace esphome {
namespace ahtx0 {

class AHTX0Component : public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void update() override;

  Adafruit_AHTX0 aht;

  void set_temperature_sensor(sensor::Sensor *temperature_sensor) { temperature_sensor_ = temperature_sensor; }
  void set_humidity_sensor(sensor::Sensor *humidity_sensor) { humidity_sensor_ = humidity_sensor; }

 protected:
  sensor::Sensor *temperature_sensor_{nullptr};
  sensor::Sensor *humidity_sensor_{nullptr};
};

}  // namespace ahtx0
}  // namespace esphome
