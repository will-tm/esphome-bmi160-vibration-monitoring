#include "bmi160_fft.h"
#include "esphome/core/log.h"
#include <cmath>
#include <algorithm>

namespace esphome {
namespace bmi160_fft {

static const char *const TAG = "bmi160_fft";

// Cooley-Tukey radix-2 FFT implementation
class FFT {
 public:
  static void compute(float *real, float *imag, size_t n) {
    size_t j = 0;
    for (size_t i = 0; i < n - 1; i++) {
      if (i < j) {
        std::swap(real[i], real[j]);
        std::swap(imag[i], imag[j]);
      }
      size_t k = n >> 1;
      while (k <= j) {
        j -= k;
        k >>= 1;
      }
      j += k;
    }

    for (size_t len = 2; len <= n; len <<= 1) {
      float angle = -2.0f * M_PI / len;
      float wpr = cosf(angle);
      float wpi = sinf(angle);

      for (size_t i = 0; i < n; i += len) {
        float wr = 1.0f;
        float wi = 0.0f;

        for (size_t jj = 0; jj < len / 2; jj++) {
          size_t a = i + jj;
          size_t b = a + len / 2;

          float tr = wr * real[b] - wi * imag[b];
          float ti = wr * imag[b] + wi * real[b];

          real[b] = real[a] - tr;
          imag[b] = imag[a] - ti;
          real[a] += tr;
          imag[a] += ti;

          float wtemp = wr;
          wr = wr * wpr - wi * wpi;
          wi = wi * wpr + wtemp * wpi;
        }
      }
    }
  }

  static void hann_window(float *data, size_t n) {
    for (size_t i = 0; i < n; i++) {
      float multiplier = 0.5f * (1.0f - cosf(2.0f * M_PI * i / (n - 1)));
      data[i] *= multiplier;
    }
  }
};

void IRAM_ATTR BMI160FFT::gpio_isr(BMI160FFT *self) {
  self->fifo_ready_ = true;
  self->isr_count_++;
}

void BMI160FFT::setup() {
  ESP_LOGI(TAG, "=== BMI160 FFT Setup Starting ===");
  ESP_LOGI(TAG, "FFT size: %d, Sample rate: %d Hz", this->fft_size_, this->sample_rate_);

  // Initialize SPI device first
  ESP_LOGI(TAG, "Initializing SPI device...");
  this->spi_setup();
  ESP_LOGI(TAG, "SPI device initialized");

  // Allocate buffers
  ESP_LOGI(TAG, "Allocating sample buffers...");
  this->samples_.resize(this->fft_size_);
  this->fft_real_.resize(this->fft_size_);
  this->fft_imag_.resize(this->fft_size_);
  this->fft_output_.resize(this->fft_size_ / 2);
  ESP_LOGI(TAG, "Buffers allocated OK");

  // Initialize BMI160
  ESP_LOGI(TAG, "Calling init_bmi160_()...");
  if (!this->init_bmi160_()) {
    ESP_LOGE(TAG, "BMI160 initialization failed - check wiring!");
    this->mark_failed();
    return;
  }

  // Configure FIFO and optionally interrupt
  ESP_LOGI(TAG, "Configuring FIFO and interrupt...");
  this->configure_fifo_and_interrupt_();

  // Initialize binary sensor to OFF (not unknown)
  if (this->running_binary_sensor_ != nullptr) {
    this->running_binary_sensor_->publish_state(false);
  }

  this->initialized_ = true;
  ESP_LOGI(TAG, "=== BMI160 FFT Setup Complete ===");
}

bool BMI160FFT::init_bmi160_() {
  ESP_LOGI(TAG, "Initializing BMI160...");

  // Wait for BMI160 to fully power up (datasheet says 3.8ms, we give 50ms)
  ESP_LOGI(TAG, "Waiting for power-up...");
  delay(50);

  // BMI160 starts in I2C mode after power-up
  // To enable SPI mode, we need to toggle CS and do a dummy read
  ESP_LOGI(TAG, "Activating SPI mode (toggle CS)...");

  // Toggle CS a few times to wake up SPI interface
  for (int i = 0; i < 5; i++) {
    this->enable();
    delayMicroseconds(10);
    this->disable();
    delayMicroseconds(10);
  }
  delay(1);

  // Dummy read to fully activate SPI mode
  this->enable();
  this->write_byte(0x80);  // Read register 0x00
  this->read_byte();
  this->disable();
  delay(1);

  // First read before reset to check SPI connectivity
  uint8_t pre_reset_id = this->read_register_(BMI160_REG_CHIP_ID);
  ESP_LOGI(TAG, "Pre-reset chip ID read: 0x%02X", pre_reset_id);

  // If still 0x00, try one more time after a longer delay
  if (pre_reset_id == 0x00) {
    ESP_LOGW(TAG, "Got 0x00, retrying after delay...");
    delay(50);
    pre_reset_id = this->read_register_(BMI160_REG_CHIP_ID);
    ESP_LOGI(TAG, "Retry chip ID read: 0x%02X", pre_reset_id);
  }

  // Soft reset
  ESP_LOGI(TAG, "Sending soft reset command...");
  this->write_register_(BMI160_REG_CMD, BMI160_CMD_SOFT_RESET);
  delay(50);  // Reset takes time

  // After reset, BMI160 defaults back to I2C mode - re-activate SPI
  ESP_LOGI(TAG, "Re-activating SPI mode after reset...");
  for (int i = 0; i < 5; i++) {
    this->enable();
    delayMicroseconds(10);
    this->disable();
    delayMicroseconds(10);
  }
  delay(1);

  // Dummy read to fully activate SPI mode
  this->enable();
  this->write_byte(0x80);  // Read register 0x00
  this->read_byte();
  this->disable();
  delay(1);

  // Check chip ID after reset
  uint8_t chip_id = this->read_register_(BMI160_REG_CHIP_ID);
  ESP_LOGI(TAG, "Post-reset chip ID: 0x%02X (expected 0x%02X)", chip_id, BMI160_CHIP_ID);

  // Also read a few more registers for debugging
  uint8_t err_reg = this->read_register_(0x02);  // ERR_REG
  uint8_t pmu_status = this->read_register_(BMI160_REG_PMU_STATUS);
  ESP_LOGI(TAG, "ERR_REG: 0x%02X, PMU_STATUS: 0x%02X", err_reg, pmu_status);

  if (chip_id != BMI160_CHIP_ID) {
    ESP_LOGE(TAG, "Wrong chip ID! Expected 0xD1.");
    if (chip_id == 0x00 || chip_id == 0xFF) {
      ESP_LOGE(TAG, "Got 0x%02X - likely no SPI response. Check wiring!", chip_id);
      ESP_LOGE(TAG, "  - Is CS connected to GPIO5?");
      ESP_LOGE(TAG, "  - Is MISO connected to GPIO19?");
      ESP_LOGE(TAG, "  - Is MOSI connected to GPIO23?");
      ESP_LOGE(TAG, "  - Is CLK connected to GPIO18?");
    }
    return false;
  }

  // Set accelerometer to normal power mode
  this->write_register_(BMI160_REG_CMD, BMI160_CMD_ACC_SET_PMU_MODE_NORMAL);
  delay(4);

  // Configure accelerometer ODR
  uint8_t odr_config;
  switch (this->sample_rate_) {
    case 100:  odr_config = 0x08; break;
    case 200:  odr_config = 0x09; break;
    case 400:  odr_config = 0x0A; break;
    case 800:  odr_config = 0x0B; break;
    case 1600: odr_config = 0x0C; break;
    default:   odr_config = 0x0C; break;
  }
  this->write_register_(BMI160_REG_ACC_CONF, (0x02 << 4) | odr_config);

  // Configure accelerometer range
  uint8_t range_config;
  switch (this->accel_range_) {
    case 2:  range_config = 0x03; this->accel_scale_ = 2.0f / 32768.0f; break;
    case 4:  range_config = 0x05; this->accel_scale_ = 4.0f / 32768.0f; break;
    case 8:  range_config = 0x08; this->accel_scale_ = 8.0f / 32768.0f; break;
    case 16: range_config = 0x0C; this->accel_scale_ = 16.0f / 32768.0f; break;
    default: range_config = 0x05; this->accel_scale_ = 4.0f / 32768.0f; break;
  }
  this->write_register_(BMI160_REG_ACC_RANGE, range_config);

  delay(10);
  uint8_t final_pmu = this->read_register_(BMI160_REG_PMU_STATUS);
  ESP_LOGI(TAG, "Final PMU Status: 0x%02X (acc_pmu=%d)", final_pmu, (final_pmu >> 4) & 0x03);

  // Check if accelerometer is in normal mode (0x01 in bits 5:4)
  if (((final_pmu >> 4) & 0x03) != 0x01) {
    ESP_LOGW(TAG, "Accelerometer not in normal mode!");
  }

  return true;
}

void BMI160FFT::configure_fifo_and_interrupt_() {
  // Flush FIFO
  this->write_register_(BMI160_REG_CMD, BMI160_CMD_FIFO_FLUSH);
  delay(1);

  // FIFO_CONFIG_0: Set watermark (in 4-byte units, so watermark * 6 / 4)
  uint8_t watermark_reg = (BMI160_FIFO_WATERMARK * BMI160_FIFO_ACC_FRAME_SIZE) / 4;
  this->write_register_(BMI160_REG_FIFO_CONFIG_0, watermark_reg);

  // FIFO_CONFIG_1: Enable accelerometer data in FIFO
  this->write_register_(BMI160_REG_FIFO_CONFIG_1, 0x40);

  if (this->interrupt_pin_ != nullptr) {
    ESP_LOGI(TAG, "Configuring interrupt on pin %d", this->interrupt_pin_->get_pin());

    // Configure interrupt pin
    this->interrupt_pin_->setup();
    this->interrupt_pin_->attach_interrupt(
        BMI160FFT::gpio_isr, this, gpio::INTERRUPT_RISING_EDGE);

    // INT_LATCH: Non-latched mode (interrupt is level-based for FIFO watermark)
    // Bits 3:0 = 0x00 for non-latched
    this->write_register_(BMI160_REG_INT_LATCH, 0x00);

    // INT_EN_1: Enable FIFO watermark interrupt
    this->write_register_(BMI160_REG_INT_EN_1, 0x40);

    // INT_OUT_CTRL: INT1 output enable, active high, push-pull
    this->write_register_(BMI160_REG_INT_OUT_CTRL, 0x08);

    // INT_MAP_1: Map FIFO watermark to INT1
    this->write_register_(BMI160_REG_INT_MAP_1, 0x40);

    ESP_LOGI(TAG, "FIFO watermark interrupt configured (non-latched)");
  } else {
    ESP_LOGI(TAG, "No interrupt pin configured, using polling mode");
  }
}

void BMI160FFT::loop() {
  // Check running timeout (even when not collecting)
  if (this->initialized_ && this->running_binary_sensor_ != nullptr && this->running_state_) {
    uint32_t now = millis();
    if (now - this->last_running_time_ >= this->running_timeout_ms_) {
      this->running_state_ = false;
      this->running_binary_sensor_->publish_state(false);
      ESP_LOGI(TAG, "Running state OFF (timeout after %d seconds)", this->running_timeout_ms_ / 1000);
    }
  }

  if (!this->initialized_ || !this->collecting_) {
    return;
  }

  // Check if we should read: ISR triggered OR periodic FIFO check (every 10ms)
  bool should_read = this->fifo_ready_;
  if (!should_read) {
    uint32_t now = millis();
    if (now - this->last_poll_time_ >= 10) {
      this->last_poll_time_ = now;
      // Check FIFO level directly (more reliable than INT pin)
      uint8_t len_data[2];
      this->read_registers_(BMI160_REG_FIFO_LENGTH, len_data, 2);
      uint16_t fifo_bytes = ((len_data[1] & 0x07) << 8) | len_data[0];
      if (fifo_bytes >= BMI160_FIFO_WATERMARK * BMI160_FIFO_ACC_FRAME_SIZE) {
        should_read = true;
      }
    }
  }

  if (!should_read) {
    return;
  }
  this->fifo_ready_ = false;

  // Read ALL available FIFO data (drain completely to clear INT)
  uint8_t len_data[2];
  this->read_registers_(BMI160_REG_FIFO_LENGTH, len_data, 2);
  uint16_t fifo_bytes = ((len_data[1] & 0x07) << 8) | len_data[0];
  uint16_t available = fifo_bytes / BMI160_FIFO_ACC_FRAME_SIZE;

  if (available == 0) {
    return;
  }

  uint16_t remaining = this->fft_size_ - this->samples_collected_;
  uint16_t to_keep = std::min(available, remaining);

  // Read samples we need
  if (to_keep > 0) {
    uint16_t read = this->read_fifo_(&this->samples_[this->samples_collected_], to_keep);
    this->samples_collected_ += read;
    ESP_LOGD(TAG, "Read %d/%d samples (FIFO had %d), total %d/%d",
             read, to_keep, available, this->samples_collected_, this->fft_size_);
  }

  // Drain any excess to get FIFO below watermark (clears INT line)
  uint16_t excess = available - to_keep;
  if (excess > 0) {
    std::vector<uint8_t> discard(excess * BMI160_FIFO_ACC_FRAME_SIZE);
    this->read_registers_(BMI160_REG_FIFO_DATA, discard.data(), discard.size());
  }

  if (this->samples_collected_ >= this->fft_size_) {
    this->collecting_ = false;
    ESP_LOGI(TAG, "Sample collection complete, running FFT");
    this->perform_fft_();
  }
}

void BMI160FFT::update() {
  if (!this->initialized_) {
    return;
  }

  if (this->interrupt_pin_ != nullptr) {
    // Interrupt-based collection
    if (this->collecting_) {
      ESP_LOGW(TAG, "Still collecting (%d/%d samples), skipping this cycle",
               this->samples_collected_, this->fft_size_);
      return;
    }

    ESP_LOGD(TAG, "Starting FFT measurement cycle");
    this->samples_collected_ = 0;
    this->collecting_ = true;
    this->fifo_ready_ = false;

    // Clear any pending interrupt status
    this->read_register_(BMI160_REG_INT_STATUS_1);

    // Flush FIFO and start fresh
    this->write_register_(BMI160_REG_CMD, BMI160_CMD_FIFO_FLUSH);
    delay(1);

    ESP_LOGD(TAG, "Waiting for FIFO interrupts to collect %d samples", this->fft_size_);
    // Collection continues in loop()
  } else {
    // Polling-based collection
    if (this->collect_samples_polling_()) {
      this->perform_fft_();
    }
  }
}

uint16_t BMI160FFT::read_fifo_(float *buffer, uint16_t max_samples) {
  // Read FIFO length
  uint8_t len_data[2];
  this->read_registers_(BMI160_REG_FIFO_LENGTH, len_data, 2);
  uint16_t fifo_bytes = (len_data[1] << 8) | len_data[0];
  fifo_bytes &= 0x07FF;  // 11-bit value

  uint16_t available_samples = fifo_bytes / BMI160_FIFO_ACC_FRAME_SIZE;
  uint16_t samples_to_read = std::min(available_samples, max_samples);

  if (samples_to_read == 0) {
    return 0;
  }

  // Read FIFO data
  uint16_t bytes_to_read = samples_to_read * BMI160_FIFO_ACC_FRAME_SIZE;
  std::vector<uint8_t> fifo_data(bytes_to_read);
  this->read_registers_(BMI160_REG_FIFO_DATA, fifo_data.data(), bytes_to_read);

  // Parse samples
  for (uint16_t i = 0; i < samples_to_read; i++) {
    uint16_t offset = i * BMI160_FIFO_ACC_FRAME_SIZE;
    int16_t raw_x = (int16_t)((fifo_data[offset + 1] << 8) | fifo_data[offset + 0]);
    int16_t raw_y = (int16_t)((fifo_data[offset + 3] << 8) | fifo_data[offset + 2]);
    int16_t raw_z = (int16_t)((fifo_data[offset + 5] << 8) | fifo_data[offset + 4]);

    float ax = raw_x * this->accel_scale_;
    float ay = raw_y * this->accel_scale_;
    float az = raw_z * this->accel_scale_;

    float magnitude = std::sqrt(ax * ax + ay * ay + az * az);
    buffer[i] = magnitude - 1.0f;  // Remove gravity
  }

  return samples_to_read;
}

bool BMI160FFT::collect_samples_polling_() {
  uint32_t sample_period_us = 1000000 / this->sample_rate_;

  ESP_LOGD(TAG, "Polling: collecting %d samples at %d Hz", this->fft_size_, this->sample_rate_);

  uint32_t start_time = micros();

  for (uint16_t i = 0; i < this->fft_size_; i++) {
    uint32_t target_time = start_time + (i * sample_period_us);
    while (micros() < target_time) {
      delayMicroseconds(1);
    }

    uint8_t data[6];
    this->read_registers_(BMI160_REG_DATA_ACC, data, 6);

    int16_t raw_x = (int16_t)((data[1] << 8) | data[0]);
    int16_t raw_y = (int16_t)((data[3] << 8) | data[2]);
    int16_t raw_z = (int16_t)((data[5] << 8) | data[4]);

    float ax = raw_x * this->accel_scale_;
    float ay = raw_y * this->accel_scale_;
    float az = raw_z * this->accel_scale_;

    float magnitude = std::sqrt(ax * ax + ay * ay + az * az);
    this->samples_[i] = magnitude - 1.0f;

    if (i % 128 == 0) {
      yield();
    }
  }

  uint32_t elapsed = micros() - start_time;
  float actual_rate = (float)this->fft_size_ / ((float)elapsed / 1000000.0f);
  ESP_LOGD(TAG, "Collected %d samples in %d us (%.1f Hz)", this->fft_size_, elapsed, actual_rate);

  return true;
}

void BMI160FFT::perform_fft_() {
  for (size_t i = 0; i < this->fft_size_; i++) {
    this->fft_real_[i] = this->samples_[i];
    this->fft_imag_[i] = 0.0f;
  }

  FFT::hann_window(this->fft_real_.data(), this->fft_size_);
  FFT::compute(this->fft_real_.data(), this->fft_imag_.data(), this->fft_size_);

  float freq_resolution = (float)this->sample_rate_ / (float)this->fft_size_;
  float peak_magnitude = 0.0f;
  size_t peak_bin = 1;
  float total_energy = 0.0f;

  for (size_t i = 1; i < this->fft_size_ / 2; i++) {
    float real = this->fft_real_[i];
    float imag = this->fft_imag_[i];
    float magnitude = std::sqrt(real * real + imag * imag);
    magnitude = (2.0f * magnitude) / this->fft_size_;

    this->fft_output_[i] = magnitude;
    total_energy += magnitude * magnitude;

    if (magnitude > peak_magnitude) {
      peak_magnitude = magnitude;
      peak_bin = i;
    }
  }

  float peak_frequency = peak_bin * freq_resolution;
  // Clamp to 0 if below frequency threshold (noise floor)
  if (peak_frequency < this->frequency_threshold_) {
    peak_frequency = 0.0f;
  }
  total_energy = std::sqrt(total_energy);

  float dominant_energy = 0.0f;
  int band_size = 0;
  for (int i = std::max(1, (int)peak_bin - 2); i <= std::min((int)(this->fft_size_ / 2 - 1), (int)peak_bin + 2); i++) {
    dominant_energy += this->fft_output_[i] * this->fft_output_[i];
    band_size++;
  }
  dominant_energy = std::sqrt(dominant_energy / band_size);

  // Calculate RPM from peak frequency (1 Hz = 60 RPM)
  float rpm = peak_frequency * 60.0f;

  ESP_LOGI(TAG, "FFT: Peak=%.2f Hz (%.1f RPM), Mag=%.4f g, Energy=%.4f",
           peak_frequency, rpm, peak_magnitude, total_energy);

  if (this->peak_frequency_sensor_ != nullptr)
    this->peak_frequency_sensor_->publish_state(peak_frequency);
  if (this->peak_magnitude_sensor_ != nullptr)
    this->peak_magnitude_sensor_->publish_state(peak_magnitude);
  if (this->total_energy_sensor_ != nullptr)
    this->total_energy_sensor_->publish_state(total_energy);
  if (this->dominant_frequency_energy_sensor_ != nullptr)
    this->dominant_frequency_energy_sensor_->publish_state(dominant_energy);
  if (this->rpm_sensor_ != nullptr)
    this->rpm_sensor_->publish_state(rpm);

  // Update running binary sensor with timeout
  if (this->running_binary_sensor_ != nullptr) {
    bool above_threshold = total_energy > this->running_threshold_;
    if (above_threshold) {
      this->last_running_time_ = millis();
      if (!this->running_state_) {
        this->running_state_ = true;
        this->running_binary_sensor_->publish_state(true);
        ESP_LOGI(TAG, "Running state ON (energy %.4f > threshold %.4f)", total_energy, this->running_threshold_);
      }
    }
    // Timeout check is done in loop()
  }
}

uint8_t BMI160FFT::read_register_(uint8_t reg) {
  this->enable();
  this->write_byte(reg | 0x80);
  uint8_t value = this->read_byte();
  this->disable();
  return value;
}

void BMI160FFT::write_register_(uint8_t reg, uint8_t value) {
  this->enable();
  this->write_byte(reg & 0x7F);
  this->write_byte(value);
  this->disable();
}

void BMI160FFT::read_registers_(uint8_t reg, uint8_t *data, size_t len) {
  this->enable();
  this->write_byte(reg | 0x80);
  this->read_array(data, len);
  this->disable();
}

void BMI160FFT::dump_config() {
  ESP_LOGCONFIG(TAG, "BMI160 FFT:");
  LOG_PIN("  CS Pin:", this->cs_);
  if (this->interrupt_pin_ != nullptr) {
    LOG_PIN("  Interrupt Pin:", this->interrupt_pin_);
  }
  ESP_LOGCONFIG(TAG, "  FFT Size: %d", this->fft_size_);
  ESP_LOGCONFIG(TAG, "  Sample Rate: %d Hz", this->sample_rate_);
  ESP_LOGCONFIG(TAG, "  Accel Range: +/-%d g", this->accel_range_);
  ESP_LOGCONFIG(TAG, "  Frequency Resolution: %.2f Hz", (float)this->sample_rate_ / this->fft_size_);
  ESP_LOGCONFIG(TAG, "  Mode: %s", this->interrupt_pin_ != nullptr ? "Interrupt" : "Polling");
  LOG_SENSOR("  ", "Peak Frequency", this->peak_frequency_sensor_);
  LOG_SENSOR("  ", "Peak Magnitude", this->peak_magnitude_sensor_);
  LOG_SENSOR("  ", "Total Energy", this->total_energy_sensor_);
  LOG_SENSOR("  ", "Dominant Frequency Energy", this->dominant_frequency_energy_sensor_);
}

// BMI160FFTNumber implementation
// Fixed preference keys for stable storage across OTA
static const uint32_t PREF_ENERGY_THRESHOLD = 0xBF160E01;
static const uint32_t PREF_TIMEOUT = 0xBF160E02;
static const uint32_t PREF_FREQ_THRESHOLD = 0xBF160E03;

void BMI160FFTNumber::setup() {
  float value;
  float restored;

  // Use fixed preference key based on type (stable across OTA)
  uint32_t pref_key;
  switch (this->type_) {
    case 0: pref_key = PREF_ENERGY_THRESHOLD; break;
    case 1: pref_key = PREF_TIMEOUT; break;
    default: pref_key = PREF_FREQ_THRESHOLD; break;
  }
  this->pref_ = global_preferences->make_preference<float>(pref_key);

  if (this->pref_.load(&restored)) {
    // Restored from flash
    value = restored;
    const char *name = this->type_ == 0 ? "energy_threshold" :
                       this->type_ == 1 ? "timeout" : "frequency_threshold";
    ESP_LOGI("bmi160_fft", "Restored %s from flash: %.4f", name, value);
    // Apply restored value
    if (this->type_ == 0) {
      this->parent_->set_running_threshold(value);
    } else if (this->type_ == 1) {
      this->parent_->set_running_timeout(value);
    } else {
      this->parent_->set_frequency_threshold(value);
    }
  } else {
    // Use default from parent
    if (this->type_ == 0) {
      value = this->parent_->get_running_threshold();
    } else if (this->type_ == 1) {
      value = this->parent_->get_running_timeout_ms() / 1000.0f;
    } else {
      value = this->parent_->get_frequency_threshold();
    }
    ESP_LOGI("bmi160_fft", "No saved value for %s, using default: %.4f",
             this->type_ == 0 ? "energy_threshold" :
             this->type_ == 1 ? "timeout" : "frequency_threshold", value);
  }

  // Store value and publish immediately and after a delay (for HA connection)
  this->state = value;
  this->publish_state(value);

  // Schedule another publish after API connects
  this->set_timeout(5000, [this, value]() {
    this->publish_state(value);
  });
}

void BMI160FFTNumber::control(float value) {
  if (this->type_ == 0) {
    // Energy threshold
    this->parent_->set_running_threshold(value);
    ESP_LOGI("bmi160_fft", "Energy threshold set to %.4f", value);
  } else if (this->type_ == 1) {
    // Timeout (in seconds)
    this->parent_->set_running_timeout(value);
    ESP_LOGI("bmi160_fft", "Running timeout set to %.0f seconds", value);
  } else {
    // Frequency threshold
    this->parent_->set_frequency_threshold(value);
    ESP_LOGI("bmi160_fft", "Frequency threshold set to %.2f Hz", value);
  }
  this->publish_state(value);

  // Save to flash and sync
  this->pref_.save(&value);
  global_preferences->sync();
}

}  // namespace bmi160_fft
}  // namespace esphome
