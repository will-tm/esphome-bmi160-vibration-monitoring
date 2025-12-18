#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/preferences.h"
#include "esphome/core/application.h"
#include "esphome/components/spi/spi.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/number/number.h"
#ifdef USE_API
#include "esphome/components/api/api_server.h"
#endif
#include <vector>

namespace esphome {
namespace bmi160_fft {

// BMI160 Register addresses
static const uint8_t BMI160_REG_CHIP_ID = 0x00;
static const uint8_t BMI160_REG_PMU_STATUS = 0x03;
static const uint8_t BMI160_REG_DATA_ACC = 0x12;
static const uint8_t BMI160_REG_STATUS = 0x1B;
static const uint8_t BMI160_REG_INT_STATUS_1 = 0x1D;
static const uint8_t BMI160_REG_FIFO_LENGTH = 0x22;
static const uint8_t BMI160_REG_FIFO_DATA = 0x24;
static const uint8_t BMI160_REG_ACC_CONF = 0x40;
static const uint8_t BMI160_REG_ACC_RANGE = 0x41;
static const uint8_t BMI160_REG_FIFO_CONFIG_0 = 0x46;
static const uint8_t BMI160_REG_FIFO_CONFIG_1 = 0x47;
static const uint8_t BMI160_REG_INT_EN_1 = 0x51;
static const uint8_t BMI160_REG_INT_OUT_CTRL = 0x53;
static const uint8_t BMI160_REG_INT_LATCH = 0x54;
static const uint8_t BMI160_REG_INT_MAP_1 = 0x56;
static const uint8_t BMI160_REG_CMD = 0x7E;

// BMI160 Commands
static const uint8_t BMI160_CMD_SOFT_RESET = 0xB6;
static const uint8_t BMI160_CMD_ACC_SET_PMU_MODE_NORMAL = 0x11;
static const uint8_t BMI160_CMD_FIFO_FLUSH = 0xB0;

// BMI160 expected chip ID
static const uint8_t BMI160_CHIP_ID = 0xD1;

// FIFO frame size for accelerometer only (no header mode)
static const uint8_t BMI160_FIFO_ACC_FRAME_SIZE = 6;

// FIFO watermark (in frames, max ~170 for 1024 byte FIFO)
// Keep low to avoid overflow - at 1600Hz, 64 samples = 40ms
static const uint8_t BMI160_FIFO_WATERMARK = 64;

// Forward declaration
class BMI160FFT;

// Number entity for configurable parameters
class BMI160FFTNumber : public number::Number, public Component {
 public:
  void set_parent(BMI160FFT *parent) { this->parent_ = parent; }
  void set_type(uint8_t type) { this->type_ = type; }
  void setup() override;
  void loop() override;
  float get_setup_priority() const override { return setup_priority::DATA - 1.0f; }  // After parent

 protected:
  void control(float value) override;
  BMI160FFT *parent_{nullptr};
  uint8_t type_{0};  // 0=energy_threshold, 1=timeout, 2=frequency_min, 3=frequency_max
  ESPPreferenceObject pref_;
  bool api_state_published_{false};
};

class BMI160FFT : public PollingComponent,
                  public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW,
                                        spi::CLOCK_PHASE_LEADING, spi::DATA_RATE_1MHZ> {
 public:
  void setup() override;
  void update() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_fft_size(uint16_t size) { this->fft_size_ = size; }
  void set_sample_rate(uint16_t rate) { this->sample_rate_ = rate; }
  void set_accel_range(uint8_t range) { this->accel_range_ = range; }
  void set_interrupt_pin(InternalGPIOPin *pin) { this->interrupt_pin_ = pin; }

  void set_peak_frequency_sensor(sensor::Sensor *sensor) { this->peak_frequency_sensor_ = sensor; }
  void set_peak_magnitude_sensor(sensor::Sensor *sensor) { this->peak_magnitude_sensor_ = sensor; }
  void set_total_energy_sensor(sensor::Sensor *sensor) { this->total_energy_sensor_ = sensor; }
  void set_dominant_frequency_energy_sensor(sensor::Sensor *sensor) { this->dominant_frequency_energy_sensor_ = sensor; }
  void set_rpm_sensor(sensor::Sensor *sensor) { this->rpm_sensor_ = sensor; }

  void set_running_binary_sensor(binary_sensor::BinarySensor *sensor) { this->running_binary_sensor_ = sensor; }
  void set_running_threshold(float threshold) { this->running_threshold_ = threshold; }
  void set_running_timeout(float timeout) { this->running_timeout_ms_ = (uint32_t)(timeout * 1000); }

  void set_energy_threshold_number(BMI160FFTNumber *number) { this->energy_threshold_number_ = number; }
  void set_timeout_number(BMI160FFTNumber *number) { this->timeout_number_ = number; }
  void set_frequency_min_number(BMI160FFTNumber *number) { this->frequency_min_number_ = number; }
  void set_frequency_max_number(BMI160FFTNumber *number) { this->frequency_max_number_ = number; }

  // Getters/setters for number controls
  float get_running_threshold() const { return this->running_threshold_; }
  uint32_t get_running_timeout_ms() const { return this->running_timeout_ms_; }
  float get_frequency_min() const { return this->frequency_min_; }
  void set_frequency_min(float freq) { this->frequency_min_ = freq; }
  float get_frequency_max() const { return this->frequency_max_; }
  void set_frequency_max(float freq) { this->frequency_max_ = freq; }

 protected:
  uint8_t read_register_(uint8_t reg);
  void write_register_(uint8_t reg, uint8_t value);
  void read_registers_(uint8_t reg, uint8_t *data, size_t len);
  bool init_bmi160_();
  void configure_fifo_and_interrupt_();
  bool collect_samples_polling_();
  uint16_t read_fifo_(float *buffer, uint16_t max_samples);
  void perform_fft_();

  static void IRAM_ATTR gpio_isr(BMI160FFT *self);

  uint16_t fft_size_{1024};
  uint16_t sample_rate_{1600};
  uint8_t accel_range_{4};
  float accel_scale_{0.0f};

  InternalGPIOPin *interrupt_pin_{nullptr};
  volatile bool fifo_ready_{false};
  volatile uint32_t isr_count_{0};
  bool collecting_{false};
  uint16_t samples_collected_{0};
  uint32_t last_poll_time_{0};

  // Sample buffers
  std::vector<float> samples_;
  std::vector<float> fft_real_;
  std::vector<float> fft_imag_;
  std::vector<float> fft_output_;

  // Sensors
  sensor::Sensor *peak_frequency_sensor_{nullptr};
  sensor::Sensor *peak_magnitude_sensor_{nullptr};
  sensor::Sensor *total_energy_sensor_{nullptr};
  sensor::Sensor *dominant_frequency_energy_sensor_{nullptr};
  sensor::Sensor *rpm_sensor_{nullptr};

  // Binary sensors
  binary_sensor::BinarySensor *running_binary_sensor_{nullptr};
  float running_threshold_{0.05f};
  uint32_t running_timeout_ms_{60000};
  uint32_t last_running_time_{0};
  bool running_state_{false};

  // Number controls
  BMI160FFTNumber *energy_threshold_number_{nullptr};
  BMI160FFTNumber *timeout_number_{nullptr};
  BMI160FFTNumber *frequency_min_number_{nullptr};
  BMI160FFTNumber *frequency_max_number_{nullptr};
  float frequency_min_{1.0f};
  float frequency_max_{800.0f};  // Default to Nyquist/2 for 1600Hz sample rate

  bool initialized_{false};
};

}  // namespace bmi160_fft
}  // namespace esphome
