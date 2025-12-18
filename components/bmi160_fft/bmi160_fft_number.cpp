#include "bmi160_fft_number.h"
#include "bmi160_fft.h"
#include "esphome/core/log.h"

namespace esphome {
namespace bmi160_fft {

namespace {
const char *const TAG = "bmi160_fft.number";

// Fixed preference keys for stable storage across OTA
const uint32_t PREF_ENERGY_THRESHOLD = 0xBF160E01;
const uint32_t PREF_TIMEOUT = 0xBF160E02;
const uint32_t PREF_FREQ_MIN = 0xBF160E03;
const uint32_t PREF_FREQ_MAX = 0xBF160E04;
}  // namespace

const char *BMI160FFTNumber::get_name() const {
  switch (type_) {
    case NumberType::ENERGY_THRESHOLD: return "energy_threshold";
    case NumberType::TIMEOUT: return "timeout";
    case NumberType::FREQUENCY_MIN: return "frequency_min";
    case NumberType::FREQUENCY_MAX: return "frequency_max";
    default: return "unknown";
  }
}

float BMI160FFTNumber::get_default_value() {
  switch (type_) {
    case NumberType::ENERGY_THRESHOLD:
      return parent_->get_energy_threshold();
    case NumberType::TIMEOUT:
      return parent_->get_timeout_seconds();
    case NumberType::FREQUENCY_MIN:
      return parent_->get_frequency_min();
    case NumberType::FREQUENCY_MAX:
      return parent_->get_frequency_max();
    default:
      return 0.0f;
  }
}

void BMI160FFTNumber::apply_value(float value) {
  switch (type_) {
    case NumberType::ENERGY_THRESHOLD:
      parent_->set_energy_threshold(value);
      break;
    case NumberType::TIMEOUT:
      parent_->set_timeout_seconds(value);
      break;
    case NumberType::FREQUENCY_MIN:
      parent_->set_frequency_min(value);
      break;
    case NumberType::FREQUENCY_MAX:
      parent_->set_frequency_max(value);
      break;
  }
}

void BMI160FFTNumber::setup() {
  // Get preference key based on type
  uint32_t pref_key;
  switch (type_) {
    case NumberType::ENERGY_THRESHOLD: pref_key = PREF_ENERGY_THRESHOLD; break;
    case NumberType::TIMEOUT: pref_key = PREF_TIMEOUT; break;
    case NumberType::FREQUENCY_MIN: pref_key = PREF_FREQ_MIN; break;
    default: pref_key = PREF_FREQ_MAX; break;
  }

  pref_ = global_preferences->make_preference<float>(pref_key);

  float value;
  if (pref_.load(&value)) {
    ESP_LOGI(TAG, "Restored %s from flash: %.4f", get_name(), value);
  } else {
    value = get_default_value();
    ESP_LOGI(TAG, "No saved value for %s, using default: %.4f", get_name(), value);
    pref_.save(&value);
  }

  apply_value(value);
  publish_state(value);
}

void BMI160FFTNumber::loop() {
  if (!api_state_published_) {
#ifdef USE_API
    if (api::global_api_server != nullptr && api::global_api_server->is_connected()) {
      publish_state(this->state);
      api_state_published_ = true;
      ESP_LOGD(TAG, "Published %s to HA: %.4f", get_name(), this->state);
    }
#else
    api_state_published_ = true;
#endif
  }
}

void BMI160FFTNumber::control(float value) {
  apply_value(value);
  ESP_LOGI(TAG, "%s set to %.4f", get_name(), value);

  publish_state(value);
  pref_.save(&value);
  global_preferences->sync();
}

}  // namespace bmi160_fft
}  // namespace esphome
