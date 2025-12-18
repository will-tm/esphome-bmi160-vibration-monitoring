#pragma once

#include <cstdint>

namespace esphome {
namespace bmi160_fft {

// BMI160 Register addresses
namespace reg {
static const uint8_t CHIP_ID = 0x00;
static const uint8_t ERR_REG = 0x02;
static const uint8_t PMU_STATUS = 0x03;
static const uint8_t DATA_ACC = 0x12;
static const uint8_t STATUS = 0x1B;
static const uint8_t INT_STATUS_1 = 0x1D;
static const uint8_t FIFO_LENGTH = 0x22;
static const uint8_t FIFO_DATA = 0x24;
static const uint8_t ACC_CONF = 0x40;
static const uint8_t ACC_RANGE = 0x41;
static const uint8_t FIFO_CONFIG_0 = 0x46;
static const uint8_t FIFO_CONFIG_1 = 0x47;
static const uint8_t INT_EN_1 = 0x51;
static const uint8_t INT_OUT_CTRL = 0x53;
static const uint8_t INT_LATCH = 0x54;
static const uint8_t INT_MAP_1 = 0x56;
static const uint8_t CMD = 0x7E;
}  // namespace reg

// BMI160 Commands
namespace cmd {
static const uint8_t SOFT_RESET = 0xB6;
static const uint8_t ACC_SET_PMU_MODE_NORMAL = 0x11;
static const uint8_t FIFO_FLUSH = 0xB0;
}  // namespace cmd

// BMI160 Constants
static const uint8_t BMI160_CHIP_ID = 0xD1;
static const uint8_t FIFO_ACC_FRAME_SIZE = 6;
static const uint8_t FIFO_WATERMARK = 64;  // In frames, max ~170 for 1024 byte FIFO

// Accelerometer range configuration
struct AccelRangeConfig {
  uint8_t reg_value;
  float scale;  // g per LSB
};

inline AccelRangeConfig get_accel_range_config(uint8_t range_g) {
  switch (range_g) {
    case 2:  return {0x03, 2.0f / 32768.0f};
    case 4:  return {0x05, 4.0f / 32768.0f};
    case 8:  return {0x08, 8.0f / 32768.0f};
    case 16: return {0x0C, 16.0f / 32768.0f};
    default: return {0x05, 4.0f / 32768.0f};
  }
}

// ODR (Output Data Rate) configuration
struct OdrConfig {
  uint8_t reg_value;
  uint16_t actual_rate;
};

inline OdrConfig get_odr_config(uint16_t requested_rate) {
  if (requested_rate <= 100) return {0x08, 100};
  if (requested_rate <= 200) return {0x09, 200};
  if (requested_rate <= 400) return {0x0A, 400};
  if (requested_rate <= 800) return {0x0B, 800};
  return {0x0C, 1600};
}

}  // namespace bmi160_fft
}  // namespace esphome
