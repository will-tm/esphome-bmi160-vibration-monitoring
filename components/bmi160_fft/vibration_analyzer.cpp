#include "vibration_analyzer.h"
#include "esphome/core/log.h"
#include <cmath>
#include <algorithm>

namespace esphome {
namespace bmi160_fft {

namespace {
const char *const TAG = "vibration_analyzer";
}

VibrationAnalyzer::VibrationAnalyzer(uint16_t fft_size, uint16_t sample_rate)
    : fft_size_(fft_size),
      sample_rate_(sample_rate),
      freq_resolution_((float)sample_rate / fft_size),
      fft_(fft_size) {
}

AnalysisResult VibrationAnalyzer::analyze(const std::vector<float> &samples) {
  AnalysisResult result{};

  // Compute FFT
  fft_.compute(samples.data(), samples.size());

  // Compute total energy (RMS of all bins, excluding DC)
  result.total_energy = compute_total_energy();
  result.is_running = result.total_energy >= config_.energy_threshold;

  // Only search for peak if above threshold
  if (result.is_running) {
    // Calculate bin range from frequency limits
    size_t min_bin = std::max((size_t)1, (size_t)std::ceil(config_.frequency_min / freq_resolution_));
    size_t max_bin = std::min((size_t)(fft_size_ / 2 - 1),
                               (size_t)std::floor(config_.frequency_max / freq_resolution_));

    ESP_LOGD(TAG, "Searching bins %zu--%zu (%.1f--%.1f Hz)",
             min_bin, max_bin, min_bin * freq_resolution_, max_bin * freq_resolution_);

    size_t peak_bin = 0;
    find_peak(min_bin, max_bin, peak_bin, result.peak_magnitude);

    if (peak_bin > 0) {
      result.peak_frequency = peak_bin * freq_resolution_;
      result.dominant_energy = compute_dominant_energy(peak_bin);
    }
  } else {
    ESP_LOGD(TAG, "Energy %.4f below threshold %.4f, skipping peak detection",
             result.total_energy, config_.energy_threshold);
  }

  result.rpm = result.peak_frequency * 60.0f;

  ESP_LOGI(TAG, "Analysis: Peak=%.2f Hz (%.1f RPM), Mag=%.4f g, Energy=%.4f, Running=%s",
           result.peak_frequency, result.rpm, result.peak_magnitude,
           result.total_energy, result.is_running ? "YES" : "NO");

  return result;
}

float VibrationAnalyzer::compute_total_energy() {
  const auto &magnitudes = fft_.get_magnitudes();
  float energy = 0.0f;

  for (size_t i = 1; i < magnitudes.size(); i++) {
    energy += magnitudes[i] * magnitudes[i];
  }

  return std::sqrt(energy);
}

void VibrationAnalyzer::find_peak(size_t min_bin, size_t max_bin,
                                   size_t &peak_bin, float &peak_magnitude) {
  const auto &magnitudes = fft_.get_magnitudes();
  peak_bin = 0;
  peak_magnitude = 0.0f;

  for (size_t i = min_bin; i <= max_bin && i < magnitudes.size(); i++) {
    if (magnitudes[i] > peak_magnitude) {
      peak_magnitude = magnitudes[i];
      peak_bin = i;
    }
  }
}

float VibrationAnalyzer::compute_dominant_energy(size_t peak_bin) {
  const auto &magnitudes = fft_.get_magnitudes();
  float energy = 0.0f;
  int count = 0;

  // RMS of ±2 bins around peak
  int start = std::max(1, (int)peak_bin - 2);
  int end = std::min((int)magnitudes.size() - 1, (int)peak_bin + 2);

  for (int i = start; i <= end; i++) {
    energy += magnitudes[i] * magnitudes[i];
    count++;
  }

  return (count > 0) ? std::sqrt(energy / count) : 0.0f;
}

}  // namespace bmi160_fft
}  // namespace esphome
