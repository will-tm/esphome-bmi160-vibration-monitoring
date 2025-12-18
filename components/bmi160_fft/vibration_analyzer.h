#pragma once

#include "fft.h"
#include <cstdint>
#include <vector>

namespace esphome {
namespace bmi160_fft {

static const size_t MAX_PEAKS = 8;

struct Peak {
  float frequency{0.0f};   // Hz
  float magnitude{0.0f};   // g
};

struct AnalysisResult {
  float peak_frequency;    // Hz (bounded by frequency_min/max)
  float peak_magnitude;    // g
  float total_energy;      // RMS of all bins
  float dominant_energy;   // RMS around peak
  float rpm;               // peak_frequency * 60

  bool is_running;         // Above energy threshold

  // Top N peaks across full spectrum (unbounded)
  Peak peaks[MAX_PEAKS];
  size_t peak_count{0};
};

struct AnalysisConfig {
  float energy_threshold{0.05f};
  float frequency_min{1.0f};
  float frequency_max{800.0f};
  size_t num_peaks{0};     // Number of peaks to find (0 = disabled)
};

class VibrationAnalyzer {
 public:
  VibrationAnalyzer(uint16_t fft_size, uint16_t sample_rate);

  // Analyze samples and return results
  AnalysisResult analyze(const std::vector<float> &samples);

  // Configuration
  void set_config(const AnalysisConfig &config) { config_ = config; }
  AnalysisConfig &get_config() { return config_; }
  const AnalysisConfig &get_config() const { return config_; }

  uint16_t get_fft_size() const { return fft_size_; }
  uint16_t get_sample_rate() const { return sample_rate_; }
  float get_frequency_resolution() const { return freq_resolution_; }

 private:
  float compute_total_energy();
  void find_peak(size_t min_bin, size_t max_bin, size_t &peak_bin, float &peak_magnitude);
  void find_n_peaks(size_t n, Peak *peaks, size_t &count);
  float compute_dominant_energy(size_t peak_bin);

  uint16_t fft_size_;
  uint16_t sample_rate_;
  float freq_resolution_;

  FFTComputer fft_;
  AnalysisConfig config_;
};

}  // namespace bmi160_fft
}  // namespace esphome
