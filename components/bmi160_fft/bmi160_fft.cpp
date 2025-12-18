#include "bmi160_fft.h"
#include "esphome/core/log.h"
#include <cmath>

namespace esphome {
namespace bmi160_fft {

namespace {
const char *const TAG = "bmi160_fft";
}

void IRAM_ATTR BMI160FFTComponent::gpio_isr(BMI160FFTComponent *self) {
  self->fifo_ready_ = true;
}

void BMI160FFTComponent::setup() {
  ESP_LOGI(TAG, "Setting up BMI160 FFT Component...");
  ESP_LOGI(TAG, "FFT size: %d, Sample rate: %d Hz", fft_size_, sample_rate_);

  // Initialize driver
  if (!driver_.initialize(sample_rate_, accel_range_)) {
    ESP_LOGE(TAG, "BMI160 initialization failed!");
    mark_failed();
    return;
  }

  // Update sample rate to actual value
  sample_rate_ = driver_.get_sample_rate();

  // Create analyzer with actual sample rate
  analyzer_ = std::make_unique<VibrationAnalyzer>(fft_size_, sample_rate_);

  // Apply num_peaks config
  if (num_peaks_ > 0) {
    analyzer_->get_config().num_peaks = num_peaks_;
    ESP_LOGI(TAG, "Peak detection enabled: %zu peaks", num_peaks_);
  }

  // Reserve sample buffer
  samples_.reserve(fft_size_);

  // Configure FIFO
  driver_.configure_fifo();

  // Configure interrupt if pin provided
  if (interrupt_pin_ != nullptr) {
    ESP_LOGI(TAG, "Configuring interrupt on GPIO%d", interrupt_pin_->get_pin());
    interrupt_pin_->setup();
    interrupt_pin_->attach_interrupt(gpio_isr, this, gpio::INTERRUPT_RISING_EDGE);
  } else {
    ESP_LOGI(TAG, "No interrupt pin, using polling mode");
  }

  // Initialize binary sensor to OFF
  if (running_binary_sensor_ != nullptr) {
    running_binary_sensor_->publish_state(false);
  }

  initialized_ = true;
  ESP_LOGI(TAG, "BMI160 FFT Component ready");
}

void BMI160FFTComponent::loop() {
  // Check running timeout
  if (initialized_ && running_binary_sensor_ != nullptr && running_state_) {
    uint32_t now = millis();
    if (now - last_running_time_ >= running_timeout_ms_) {
      running_state_ = false;
      running_binary_sensor_->publish_state(false);
      ESP_LOGI(TAG, "Running state OFF (timeout)");
    }
  }

  if (!initialized_ || !collecting_) {
    return;
  }

  // Check if we should read FIFO
  bool should_read = fifo_ready_;
  if (!should_read) {
    uint32_t now = millis();
    if (now - last_poll_time_ >= 10) {
      last_poll_time_ = now;
      if (driver_.get_fifo_sample_count() >= FIFO_WATERMARK) {
        should_read = true;
      }
    }
  }

  if (!should_read) return;
  fifo_ready_ = false;

  // Read available samples
  uint16_t available = driver_.get_fifo_sample_count();
  if (available == 0) return;

  uint16_t remaining = fft_size_ - samples_.size();
  uint16_t to_read = std::min(available, remaining);

  if (to_read > 0) {
    size_t old_size = samples_.size();
    driver_.read_fifo_samples(samples_, to_read);
    ESP_LOGD(TAG, "Read %d samples, total %d/%d",
             (int)(samples_.size() - old_size), (int)samples_.size(), fft_size_);
  }

  // Drain excess to clear interrupt
  uint16_t excess = available - to_read;
  if (excess > 0) {
    driver_.drain_fifo(excess);
  }

  // Check if collection complete
  if (samples_.size() >= fft_size_) {
    collecting_ = false;
    ESP_LOGI(TAG, "Sample collection complete, analyzing...");

    auto result = analyzer_->analyze(samples_);
    publish_results(result);
    update_running_state(result.is_running);
  }
}

void BMI160FFTComponent::update() {
  if (!initialized_) return;

  if (collecting_) {
    ESP_LOGW(TAG, "Still collecting, skipping this cycle");
    return;
  }

  ESP_LOGD(TAG, "Starting measurement cycle");
  samples_.clear();
  collecting_ = true;
  fifo_ready_ = false;

  driver_.clear_interrupt();
  driver_.flush_fifo();

  // If no interrupt, use polling mode
  if (interrupt_pin_ == nullptr) {
    collect_samples_polling();
  }
}

void BMI160FFTComponent::collect_samples_polling() {
  uint32_t sample_period_us = 1000000 / sample_rate_;
  uint32_t start_time = micros();

  ESP_LOGD(TAG, "Polling mode: collecting %d samples", fft_size_);

  for (uint16_t i = 0; i < fft_size_; i++) {
    uint32_t target_time = start_time + (i * sample_period_us);
    while (micros() < target_time) {
      delayMicroseconds(1);
    }

    auto sample = driver_.read_accel_direct();
    samples_.push_back(sample.magnitude() - 1.0f);

    if (i % 128 == 0) yield();
  }

  collecting_ = false;
  auto result = analyzer_->analyze(samples_);
  publish_results(result);
  update_running_state(result.is_running);
}

void BMI160FFTComponent::publish_results(const AnalysisResult &result) {
  if (peak_frequency_sensor_) peak_frequency_sensor_->publish_state(result.peak_frequency);
  if (peak_magnitude_sensor_) peak_magnitude_sensor_->publish_state(result.peak_magnitude);
  if (total_energy_sensor_) total_energy_sensor_->publish_state(result.total_energy);
  if (dominant_frequency_energy_sensor_) dominant_frequency_energy_sensor_->publish_state(result.dominant_energy);
  if (rpm_sensor_) rpm_sensor_->publish_state(result.rpm);

  // Publish N peaks
  for (size_t i = 0; i < MAX_PEAKS; i++) {
    if (peak_frequency_sensors_[i]) {
      peak_frequency_sensors_[i]->publish_state(result.peaks[i].frequency);
    }
    if (peak_magnitude_sensors_[i]) {
      peak_magnitude_sensors_[i]->publish_state(result.peaks[i].magnitude);
    }
  }
}

void BMI160FFTComponent::update_running_state(bool is_running) {
  if (running_binary_sensor_ == nullptr) return;

  if (is_running) {
    last_running_time_ = millis();
    if (!running_state_) {
      running_state_ = true;
      running_binary_sensor_->publish_state(true);
      ESP_LOGI(TAG, "Running state ON");
    }
  }
}

float BMI160FFTComponent::get_energy_threshold() const {
  if (analyzer_) return analyzer_->get_config().energy_threshold;
  return 0.05f;
}

void BMI160FFTComponent::set_energy_threshold(float value) {
  if (analyzer_) analyzer_->get_config().energy_threshold = value;
}

float BMI160FFTComponent::get_frequency_min() const {
  if (analyzer_) return analyzer_->get_config().frequency_min;
  return 1.0f;
}

void BMI160FFTComponent::set_frequency_min(float value) {
  if (analyzer_) analyzer_->get_config().frequency_min = value;
}

float BMI160FFTComponent::get_frequency_max() const {
  if (analyzer_) return analyzer_->get_config().frequency_max;
  return 800.0f;
}

void BMI160FFTComponent::set_frequency_max(float value) {
  if (analyzer_) analyzer_->get_config().frequency_max = value;
}

void BMI160FFTComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "BMI160 FFT Component:");
  ESP_LOGCONFIG(TAG, "  FFT Size: %d", fft_size_);
  ESP_LOGCONFIG(TAG, "  Sample Rate: %d Hz", sample_rate_);
  ESP_LOGCONFIG(TAG, "  Accel Range: +/-%d g", accel_range_);
  ESP_LOGCONFIG(TAG, "  Frequency Resolution: %.2f Hz", (float)sample_rate_ / fft_size_);
  ESP_LOGCONFIG(TAG, "  Mode: %s", interrupt_pin_ != nullptr ? "Interrupt" : "Polling");

  driver_.dump_config();

  LOG_SENSOR("  ", "Peak Frequency", peak_frequency_sensor_);
  LOG_SENSOR("  ", "Peak Magnitude", peak_magnitude_sensor_);
  LOG_SENSOR("  ", "Total Energy", total_energy_sensor_);
  LOG_SENSOR("  ", "Dominant Energy", dominant_frequency_energy_sensor_);
  LOG_SENSOR("  ", "RPM", rpm_sensor_);
  LOG_BINARY_SENSOR("  ", "Running", running_binary_sensor_);

  // Log peak sensors
  for (size_t i = 0; i < MAX_PEAKS; i++) {
    if (peak_frequency_sensors_[i]) {
      ESP_LOGCONFIG(TAG, "  Peak %zu Frequency: %s", i + 1, peak_frequency_sensors_[i]->get_name().c_str());
    }
    if (peak_magnitude_sensors_[i]) {
      ESP_LOGCONFIG(TAG, "  Peak %zu Magnitude: %s", i + 1, peak_magnitude_sensors_[i]->get_name().c_str());
    }
  }
}

}  // namespace bmi160_fft
}  // namespace esphome
