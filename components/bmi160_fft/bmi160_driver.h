#pragma once

#include "esphome/core/hal.h"
#include "esphome/components/spi/spi.h"
#include "bmi160_registers.h"
#include <vector>

namespace esphome {
namespace bmi160_fft {

struct AccelSample {
  float x, y, z;
  float magnitude() const;
};

class BMI160Driver : public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW,
                                            spi::CLOCK_PHASE_LEADING, spi::DATA_RATE_1MHZ> {
 public:
  bool initialize(uint16_t sample_rate, uint8_t accel_range);
  void configure_fifo();
  void flush_fifo();

  // Read samples from FIFO, returns number of samples read
  uint16_t read_fifo_samples(std::vector<float> &buffer, uint16_t max_samples);

  // Read current accelerometer data directly (for polling mode)
  AccelSample read_accel_direct();

  // Get FIFO fill level in samples
  uint16_t get_fifo_sample_count();

  // Drain excess FIFO samples (to clear interrupt)
  void drain_fifo(uint16_t count);

  // Clear interrupt status
  void clear_interrupt();

  uint16_t get_sample_rate() const { return actual_sample_rate_; }
  float get_accel_scale() const { return accel_scale_; }

  void dump_config();

 protected:
  uint8_t read_register(uint8_t reg);
  void write_register(uint8_t reg, uint8_t value);
  void read_registers(uint8_t reg, uint8_t *data, size_t len);

 private:
  bool activate_spi_mode();
  bool verify_chip_id();
  bool configure_accelerometer(uint16_t sample_rate, uint8_t accel_range);

  uint16_t actual_sample_rate_{1600};
  float accel_scale_{0.0f};
};

}  // namespace bmi160_fft
}  // namespace esphome
