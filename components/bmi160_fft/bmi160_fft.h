#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "bmi160_driver.h"
#include "vibration_analyzer.h"
#include "bmi160_fft_number.h"
#include <vector>
#include <memory>

namespace esphome {
namespace bmi160_fft {

class BMI160FFTComponent : public PollingComponent {
 public:
  void setup() override;
  void update() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // Configuration setters (called from Python)
  void set_cs_pin(GPIOPin *pin) { cs_pin_ = pin; }
  void set_interrupt_pin(InternalGPIOPin *pin) { interrupt_pin_ = pin; }
  void set_fft_size(uint16_t size) { fft_size_ = size; }
  void set_sample_rate(uint16_t rate) { sample_rate_ = rate; }
  void set_accel_range(uint8_t range) { accel_range_ = range; }

  // Sensor setters
  void set_peak_frequency_sensor(sensor::Sensor *s) { peak_frequency_sensor_ = s; }
  void set_peak_magnitude_sensor(sensor::Sensor *s) { peak_magnitude_sensor_ = s; }
  void set_total_energy_sensor(sensor::Sensor *s) { total_energy_sensor_ = s; }
  void set_dominant_frequency_energy_sensor(sensor::Sensor *s) { dominant_frequency_energy_sensor_ = s; }
  void set_rpm_sensor(sensor::Sensor *s) { rpm_sensor_ = s; }

  // Peak sensor setters (for N peaks feature)
  void set_peak_frequency_sensor(size_t index, sensor::Sensor *s) {
    if (index < MAX_PEAKS) peak_frequency_sensors_[index] = s;
  }
  void set_peak_magnitude_sensor(size_t index, sensor::Sensor *s) {
    if (index < MAX_PEAKS) peak_magnitude_sensors_[index] = s;
  }
  void set_num_peaks(size_t count) { num_peaks_ = count; }

  // Binary sensor setters
  void set_running_binary_sensor(binary_sensor::BinarySensor *s) { running_binary_sensor_ = s; }

  // Number entity setters
  void set_energy_threshold_number(BMI160FFTNumber *n) { energy_threshold_number_ = n; }
  void set_timeout_number(BMI160FFTNumber *n) { timeout_number_ = n; }
  void set_frequency_min_number(BMI160FFTNumber *n) { frequency_min_number_ = n; }
  void set_frequency_max_number(BMI160FFTNumber *n) { frequency_max_number_ = n; }

  // Config getters/setters for Number entities
  float get_energy_threshold() const;
  void set_energy_threshold(float value);
  float get_timeout_seconds() const { return running_timeout_ms_ / 1000.0f; }
  void set_timeout_seconds(float value) { running_timeout_ms_ = (uint32_t)(value * 1000); }
  float get_frequency_min() const;
  void set_frequency_min(float value);
  float get_frequency_max() const;
  void set_frequency_max(float value);

  // Access to driver for SPI registration
  BMI160Driver &get_driver() { return driver_; }

 protected:
  static void IRAM_ATTR gpio_isr(BMI160FFTComponent *self);
  void collect_samples_polling();
  void publish_results(const AnalysisResult &result);
  void update_running_state(bool is_running);

  // Configuration
  GPIOPin *cs_pin_{nullptr};
  InternalGPIOPin *interrupt_pin_{nullptr};
  uint16_t fft_size_{1024};
  uint16_t sample_rate_{1600};
  uint8_t accel_range_{4};
  size_t num_peaks_{0};

  // Components
  BMI160Driver driver_;
  std::unique_ptr<VibrationAnalyzer> analyzer_;

  // Sample collection state
  std::vector<float> samples_;
  volatile bool fifo_ready_{false};
  bool collecting_{false};
  uint16_t samples_collected_{0};
  uint32_t last_poll_time_{0};

  // Sensors
  sensor::Sensor *peak_frequency_sensor_{nullptr};
  sensor::Sensor *peak_magnitude_sensor_{nullptr};
  sensor::Sensor *total_energy_sensor_{nullptr};
  sensor::Sensor *dominant_frequency_energy_sensor_{nullptr};
  sensor::Sensor *rpm_sensor_{nullptr};

  // Peak sensors (N peaks feature)
  sensor::Sensor *peak_frequency_sensors_[MAX_PEAKS]{};
  sensor::Sensor *peak_magnitude_sensors_[MAX_PEAKS]{};

  // Binary sensors
  binary_sensor::BinarySensor *running_binary_sensor_{nullptr};
  uint32_t running_timeout_ms_{60000};
  uint32_t last_running_time_{0};
  bool running_state_{false};

  // Number entities
  BMI160FFTNumber *energy_threshold_number_{nullptr};
  BMI160FFTNumber *timeout_number_{nullptr};
  BMI160FFTNumber *frequency_min_number_{nullptr};
  BMI160FFTNumber *frequency_max_number_{nullptr};

  bool initialized_{false};
};

}  // namespace bmi160_fft
}  // namespace esphome
