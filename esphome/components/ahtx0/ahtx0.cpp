// Implementation based on:
//  - Adafruit AHTX0: https://github.com/adafruit/Adafruit_AHTX0
//  - https://gist.github.com/alvesvaren/54e26a23f881b9914e0f29bdb29ef6ee
//  - Datasheet (en):
//      https://files.seeedstudio.com/wiki/Grove-AHT20_I2C_Industrial_Grade_Temperature_and_Humidity_Sensor/AHT20-datasheet-2020-4-16.pdf


#include "ahtx0.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace ahtx0 {

static const char *const TAG = "ahtx0";

void AHTX0Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up AHTX0Sensor...");
  if (!aht.begin()) {
      ESP_LOGCONFIG(TAG, "Failed to start AHTX0Sensor...");
  }
  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
  LOG_SENSOR("  ", "Humidity", this->humidity_sensor_);
}

void AHTX0Component::update() {
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);
  temperature_sensor_->publish_state(temp.temperature);
  humidity_sensor_->publish_state(humidity.relative_humidity);
}

}  // namespace ahtx0
}  // namespace esphome
