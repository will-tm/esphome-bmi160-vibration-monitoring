#pragma once

#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#include "esphome/components/number/number.h"
#ifdef USE_API
#include "esphome/components/api/api_server.h"
#endif

namespace esphome {
namespace bmi160_fft {

// Forward declaration
class BMI160FFTComponent;

enum class NumberType : uint8_t {
  ENERGY_THRESHOLD = 0,
  TIMEOUT = 1,
  FREQUENCY_MIN = 2,
  FREQUENCY_MAX = 3,
};

class BMI160FFTNumber : public number::Number, public Component {
 public:
  void set_parent(BMI160FFTComponent *parent) { parent_ = parent; }
  void set_number_type(uint8_t type) { type_ = static_cast<NumberType>(type); }

  void setup() override;
  void loop() override;
  float get_setup_priority() const override { return setup_priority::DATA - 1.0f; }

 protected:
  void control(float value) override;

 private:
  float get_default_value();
  void apply_value(float value);
  const char *get_name() const;

  BMI160FFTComponent *parent_{nullptr};
  NumberType type_{NumberType::ENERGY_THRESHOLD};
  ESPPreferenceObject pref_;
  bool api_state_published_{false};
};

}  // namespace bmi160_fft
}  // namespace esphome
