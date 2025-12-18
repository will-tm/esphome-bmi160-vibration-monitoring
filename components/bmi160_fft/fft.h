#pragma once

#include <cstddef>
#include <vector>

namespace esphome {
namespace bmi160_fft {

class FFTComputer {
 public:
  explicit FFTComputer(size_t size);

  // Compute FFT on input data, results stored in magnitudes_
  void compute(const float *input, size_t length);

  // Get magnitude at bin index
  float get_magnitude(size_t bin) const;

  // Get all magnitudes (size/2 elements)
  const std::vector<float> &get_magnitudes() const { return magnitudes_; }

  size_t get_size() const { return size_; }

 private:
  void apply_hann_window(float *data, size_t n);
  void compute_fft(float *real, float *imag, size_t n);

  size_t size_;
  std::vector<float> real_;
  std::vector<float> imag_;
  std::vector<float> magnitudes_;
};

}  // namespace bmi160_fft
}  // namespace esphome
