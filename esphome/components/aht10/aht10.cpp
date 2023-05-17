#include "aht10.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace aht20 {

static const char *const TAG = "aht20";
static const uint8_t AHT20_CALIBRATE_CMD[] = {0xBE};
static const uint8_t AHT20_MEASURE_CMD[] = {0xAC, 0x33, 0x00};
static const uint8_t AHT20_INIT_DELAY = 40;
static const uint8_t AHT20_DEFAULT_DELAY = 5;
static const uint8_t AHT20_HUMIDITY_DELAY = 30;
static const uint8_t AHT20_ATTEMPTS = 3;

class AHT10Component : public PollingComponent, public sensor::Sensor {
 public:
  void setup() {
    ESP_LOGCONFIG(TAG, "Setting up AHT20...");

    delay(AHT20_INIT_DELAY);

    if (!this->write_bytes(0, AHT20_CALIBRATE_CMD, sizeof(AHT20_CALIBRATE_CMD))) {
      ESP_LOGE(TAG, "Communication with AHT20 failed!");
      this->mark_failed();
      return;
    }
    uint8_t data = 0;
    if (this->write(&data, 1) != i2c::ERROR_OK) {
      ESP_LOGD(TAG, "Communication with AHT20 failed!");
      this->mark_failed();
      return;
    }
    delay(AHT20_DEFAULT_DELAY);
    if (this->read(&data, 1) != i2c::ERROR_OK) {
      ESP_LOGD(TAG, "Communication with AHT20 failed!");
      this->mark_failed();
      return;
    }
    if (this->read(&data, 1) != i2c::ERROR_OK) {
      ESP_LOGD(TAG, "Communication with AHT20 failed!");
      this->mark_failed();
      return;
    }
    if ((data & 0x68) != 0x08) {
      ESP_LOGE(TAG, "AHT20 calibration failed!");
      this->mark_failed();
      return;
    }

    ESP_LOGV(TAG, "AHT20 calibrated");
  }

  void update() {
    if (!this->write_bytes(0, AHT20_MEASURE_CMD, sizeof(AHT20_MEASURE_CMD))) {
      ESP_LOGE(TAG, "Communication with AHT20 failed!");
      this->status_set_warning();
      return;
    }
    if (!this->write_bytes(0, AHT10_MEASURE_CMD, sizeof(AHT10_MEASURE_CMD))) {
      ESP_LOGE(TAG, "Communication with AHT10 failed!");
      this->status_set_warning();
      return;
    }
    uint8_t data[6];
    uint8_t delay_ms = AHT10_DEFAULT_DELAY;
    if (this->humidity_sensor_ != nullptr)
      delay_ms = AHT10_HUMIDITY_DELAY;
    bool success = false;
    for (int i = 0; i < AHT10_ATTEMPTS; ++i) {
      ESP_LOGVV(TAG, "Attempt %d at %6u", i, millis());
      delay(delay_ms);
      if (this->read(data, 6) != i2c::ERROR_OK) {
        ESP_LOGD(TAG, "Communication with AHT10 failed, waiting...");
        continue;
      }

      if ((data[0] & 0x80) == 0x80) {  // Bit[7] = 0b1, device is busy
        ESP_LOGD(TAG, "AHT10 is busy, waiting...");
      } else if (data[1] == 0x0 && data[2] == 0x0 && (data[3] >> 4) == 0x0) {
        // Unrealistic humidity (0x0)
        if (this->humidity_sensor_ == nullptr) {
          ESP_LOGVV(TAG, "ATH10 Unrealistic humidity (0x0), but humidity is not required");
          break;
        } else {
          ESP_LOGD(TAG, "ATH10 Unrealistic humidity (0x0), retrying...");
          if (!this->write_bytes(0, AHT10_MEASURE_CMD, sizeof(AHT10_MEASURE_CMD))) {
            ESP_LOGE(TAG, "Communication with AHT10 failed!");
            this->status_set_warning();
            return;
          }
        }
      } else {
        // data is valid, we can break the loop
        ESP_LOGVV(TAG, "Answer at %6u", millis());
        success = true;
        break;
      }
    }
    if (!success || (data[0] & 0x80) == 0x80) {
      ESP_LOGE(TAG, "Measurements reading timed-out!");
      this->status_set_warning();
      return;
    }

    uint32_t raw_temperature = ((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5];
    uint32_t raw_humidity = ((data[1] << 16) | (data[2] << 8) | data[3]) >> 4;

    float temperature = ((200.0f * (float) raw_temperature) / 1048576.0f) - 50.0f;
    float humidity;
    if (raw_humidity == 0) {  // unrealistic value
      humidity = NAN;
    } else {
      humidity = (float) raw_humidity * 100.0f / 1048576.0f;
    }

    if (this->temperature_sensor_ != nullptr) {
      this->temperature_sensor_->publish_state(temperature);
    }
    if (this->humidity_sensor_ != nullptr) {
      if (std::isnan(humidity)) {
        ESP_LOGW(TAG, "Invalid humidity! Sensor reported 0%% Hum");
      }
      this->humidity_sensor_->publish_state(humidity);
    }
    this->status_clear_warning();
  }
  float get_setup_priority() const { return setup_priority::DATA; }

  void dump_config() {
    ESP_LOGCONFIG(TAG, "AHT10:");
    LOG_I2C_DEVICE(this);
    if (this->is_failed()) {
      ESP_LOGE(TAG, "Communication with AHT10 failed!");
    }
    LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
    LOG_SENSOR("  ", "Humidity", this->humidity_sensor_);
  }
};

}  // namespace aht20
}  // namespace esphome
