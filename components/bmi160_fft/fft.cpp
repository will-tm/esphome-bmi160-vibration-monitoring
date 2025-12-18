#include "fft.h"
#include <cmath>
#include <algorithm>

namespace esphome {
namespace bmi160_fft {

FFTComputer::FFTComputer(size_t size) : size_(size) {
  real_.resize(size);
  imag_.resize(size);
  magnitudes_.resize(size / 2);
}

void FFTComputer::apply_hann_window(float *data, size_t n) {
  for (size_t i = 0; i < n; i++) {
    float multiplier = 0.5f * (1.0f - cosf(2.0f * M_PI * i / (n - 1)));
    data[i] *= multiplier;
  }
}

void FFTComputer::compute_fft(float *real, float *imag, size_t n) {
  // Cooley-Tukey radix-2 FFT
  // Bit-reversal permutation
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

  // FFT computation
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

void FFTComputer::compute(const float *input, size_t length) {
  // Copy input to real buffer, zero imaginary
  for (size_t i = 0; i < size_; i++) {
    real_[i] = (i < length) ? input[i] : 0.0f;
    imag_[i] = 0.0f;
  }

  // Apply window and compute FFT
  apply_hann_window(real_.data(), size_);
  compute_fft(real_.data(), imag_.data(), size_);

  // Compute magnitudes (normalized)
  for (size_t i = 0; i < size_ / 2; i++) {
    float mag = std::sqrt(real_[i] * real_[i] + imag_[i] * imag_[i]);
    magnitudes_[i] = (2.0f * mag) / size_;
  }
}

float FFTComputer::get_magnitude(size_t bin) const {
  if (bin >= magnitudes_.size()) return 0.0f;
  return magnitudes_[bin];
}

}  // namespace bmi160_fft
}  // namespace esphome
