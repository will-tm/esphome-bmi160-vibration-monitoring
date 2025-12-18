#include "bmi160_driver.h"
#include "esphome/core/log.h"
#include <cmath>

namespace esphome {
namespace bmi160_fft {

namespace {
const char *const TAG = "bmi160_driver";
}

float AccelSample::magnitude() const {
  return std::sqrt(x * x + y * y + z * z);
}

bool BMI160Driver::initialize(uint16_t sample_rate, uint8_t accel_range) {
  ESP_LOGI(TAG, "Initializing BMI160...");

  this->spi_setup();
  delay(50);  // Wait for power-up

  if (!activate_spi_mode()) {
    return false;
  }

  if (!verify_chip_id()) {
    return false;
  }

  // Soft reset
  write_register(reg::CMD, cmd::SOFT_RESET);
  delay(50);

  // Re-activate SPI mode after reset
  if (!activate_spi_mode()) {
    return false;
  }

  if (!configure_accelerometer(sample_rate, accel_range)) {
    return false;
  }

  ESP_LOGI(TAG, "BMI160 initialized successfully");
  return true;
}

bool BMI160Driver::activate_spi_mode() {
  // BMI160 starts in I2C mode - toggle CS to enable SPI
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

  return true;
}

bool BMI160Driver::verify_chip_id() {
  uint8_t chip_id = read_register(reg::CHIP_ID);
  ESP_LOGI(TAG, "Chip ID: 0x%02X (expected 0x%02X)", chip_id, BMI160_CHIP_ID);

  if (chip_id != BMI160_CHIP_ID) {
    ESP_LOGE(TAG, "Wrong chip ID! Check SPI wiring.");
    return false;
  }
  return true;
}

bool BMI160Driver::configure_accelerometer(uint16_t sample_rate, uint8_t accel_range) {
  // Set accelerometer to normal power mode
  write_register(reg::CMD, cmd::ACC_SET_PMU_MODE_NORMAL);
  delay(4);

  // Configure ODR
  auto odr = get_odr_config(sample_rate);
  actual_sample_rate_ = odr.actual_rate;
  if (sample_rate != odr.actual_rate) {
    ESP_LOGW(TAG, "Requested %d Hz, using %d Hz", sample_rate, odr.actual_rate);
  }

  uint8_t acc_conf_value = (0x02 << 4) | odr.reg_value;  // BWP=normal, ODR=configured
  write_register(reg::ACC_CONF, acc_conf_value);

  // Configure range
  auto range_cfg = get_accel_range_config(accel_range);
  accel_scale_ = range_cfg.scale;
  write_register(reg::ACC_RANGE, range_cfg.reg_value);

  delay(10);

  // Verify PMU status
  uint8_t pmu_status = read_register(reg::PMU_STATUS);
  uint8_t acc_pmu = (pmu_status >> 4) & 0x03;
  if (acc_pmu != 0x01) {
    ESP_LOGW(TAG, "Accelerometer not in normal mode (PMU=0x%02X)", pmu_status);
  }

  ESP_LOGI(TAG, "Configured: ODR=%d Hz, Range=+/-%dg, Scale=%.6f g/LSB",
           actual_sample_rate_, accel_range, accel_scale_);
  return true;
}

void BMI160Driver::configure_fifo() {
  flush_fifo();

  // Configure FIFO watermark
  uint8_t watermark_reg = (FIFO_WATERMARK * FIFO_ACC_FRAME_SIZE) / 4;
  write_register(reg::FIFO_CONFIG_0, watermark_reg);

  // Enable accelerometer data in FIFO
  write_register(reg::FIFO_CONFIG_1, 0x40);

  // Configure interrupt registers (actual pin attachment done by Component)
  write_register(reg::INT_LATCH, 0x00);      // Non-latched
  write_register(reg::INT_EN_1, 0x40);       // Enable FIFO watermark
  write_register(reg::INT_OUT_CTRL, 0x08);   // INT1 active high, push-pull
  write_register(reg::INT_MAP_1, 0x40);      // Map FIFO watermark to INT1

  ESP_LOGI(TAG, "FIFO configured");
}

void BMI160Driver::flush_fifo() {
  write_register(reg::CMD, cmd::FIFO_FLUSH);
  delay(1);
}

uint16_t BMI160Driver::get_fifo_sample_count() {
  uint8_t len_data[2];
  read_registers(reg::FIFO_LENGTH, len_data, 2);
  uint16_t fifo_bytes = ((len_data[1] & 0x07) << 8) | len_data[0];
  return fifo_bytes / FIFO_ACC_FRAME_SIZE;
}

uint16_t BMI160Driver::read_fifo_samples(std::vector<float> &buffer, uint16_t max_samples) {
  uint16_t available = get_fifo_sample_count();
  uint16_t to_read = std::min(available, max_samples);

  if (to_read == 0) return 0;

  uint16_t bytes_to_read = to_read * FIFO_ACC_FRAME_SIZE;
  std::vector<uint8_t> raw_data(bytes_to_read);
  read_registers(reg::FIFO_DATA, raw_data.data(), bytes_to_read);

  // Parse and compute magnitude for each sample
  for (uint16_t i = 0; i < to_read; i++) {
    uint16_t offset = i * FIFO_ACC_FRAME_SIZE;
    int16_t raw_x = (int16_t)((raw_data[offset + 1] << 8) | raw_data[offset + 0]);
    int16_t raw_y = (int16_t)((raw_data[offset + 3] << 8) | raw_data[offset + 2]);
    int16_t raw_z = (int16_t)((raw_data[offset + 5] << 8) | raw_data[offset + 4]);

    float ax = raw_x * accel_scale_;
    float ay = raw_y * accel_scale_;
    float az = raw_z * accel_scale_;

    float magnitude = std::sqrt(ax * ax + ay * ay + az * az);
    buffer.push_back(magnitude - 1.0f);  // Remove gravity
  }

  return to_read;
}

AccelSample BMI160Driver::read_accel_direct() {
  uint8_t data[6];
  read_registers(reg::DATA_ACC, data, 6);

  int16_t raw_x = (int16_t)((data[1] << 8) | data[0]);
  int16_t raw_y = (int16_t)((data[3] << 8) | data[2]);
  int16_t raw_z = (int16_t)((data[5] << 8) | data[4]);

  return AccelSample{
    raw_x * accel_scale_,
    raw_y * accel_scale_,
    raw_z * accel_scale_
  };
}

void BMI160Driver::drain_fifo(uint16_t count) {
  if (count == 0) return;
  std::vector<uint8_t> discard(count * FIFO_ACC_FRAME_SIZE);
  read_registers(reg::FIFO_DATA, discard.data(), discard.size());
}

void BMI160Driver::clear_interrupt() {
  read_register(reg::INT_STATUS_1);
}

uint8_t BMI160Driver::read_register(uint8_t reg) {
  this->enable();
  this->write_byte(reg | 0x80);
  uint8_t value = this->read_byte();
  this->disable();
  return value;
}

void BMI160Driver::write_register(uint8_t reg, uint8_t value) {
  this->enable();
  this->write_byte(reg & 0x7F);
  this->write_byte(value);
  this->disable();
}

void BMI160Driver::read_registers(uint8_t reg, uint8_t *data, size_t len) {
  this->enable();
  this->write_byte(reg | 0x80);
  this->read_array(data, len);
  this->disable();
}

void BMI160Driver::dump_config() {
  ESP_LOGCONFIG(TAG, "BMI160 Driver:");
  ESP_LOGCONFIG(TAG, "  Sample Rate: %d Hz", actual_sample_rate_);
  ESP_LOGCONFIG(TAG, "  Accel Scale: %.6f g/LSB", accel_scale_);
}

}  // namespace bmi160_fft
}  // namespace esphome
