# ESPHome BMI160 Vibration Monitoring Component

A custom ESPHome component for vibration monitoring using the BMI160 accelerometer with FFT (Fast Fourier Transform) analysis. Perfect for detecting appliance operation states (washing machines, dryers, HVAC systems) or monitoring rotating machinery.

## Features

- **FFT-based vibration analysis** - Performs real-time frequency domain analysis of accelerometer data
- **Peak frequency detection** - Identifies the dominant vibration frequency
- **RPM calculation** - Converts peak frequency to rotations per minute
- **Energy-based running detection** - Binary sensor that indicates when vibration exceeds a threshold
- **Configurable parameters** - Adjust thresholds via Home Assistant number entities
- **Interrupt or polling mode** - Use hardware interrupt for efficient data collection or polling for simpler setups
- **Persistent settings** - Threshold values are saved to flash and persist across reboots

## Hardware Requirements

- ESP32 or ESP8266 board
- BMI160 accelerometer module (SPI interface)

### Wiring (ESP32 example)

| BMI160 Pin | ESP32 Pin |
|------------|-----------|
| VCC        | 3.3V      |
| GND        | GND       |
| SCL/SCLK   | GPIO18    |
| SDA/MOSI   | GPIO23    |
| SDO/MISO   | GPIO19    |
| CS         | GPIO5     |
| INT1       | GPIO4 (optional) |

## Installation

Add this repository as an external component in your ESPHome configuration:

```yaml
external_components:
  - source: github://will-tm/esphome-bmi160-vibration-monitoring
    components: [bmi160_fft]
```

## Configuration

### Basic Configuration

```yaml
spi:
  clk_pin: GPIO18
  mosi_pin: GPIO23
  miso_pin: GPIO19

bmi160_fft:
  id: vibration_sensor
  cs_pin: GPIO5
  update_interval: 5s

sensor:
  - platform: bmi160_fft
    peak_frequency:
      name: "Vibration Peak Frequency"
    peak_magnitude:
      name: "Vibration Peak Magnitude"
    total_energy:
      name: "Vibration Total Energy"
    rpm:
      name: "Vibration RPM"

binary_sensor:
  - platform: bmi160_fft
    running:
      name: "Appliance Running"
      energy_threshold: 0.05
      timeout: 60s
```

### Configuration Variables

#### `bmi160_fft` Platform

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `cs_pin` | pin | **Required** | SPI chip select pin |
| `fft_size` | int | 1024 | FFT window size (128, 256, 512, 1024, 2048, 4096) |
| `sample_rate` | int | 1600 | Accelerometer sample rate in Hz (100, 200, 400, 800, 1600) |
| `accel_range` | int | 4 | Accelerometer range in g (2, 4, 8, 16) |
| `interrupt_pin` | pin | Optional | GPIO pin connected to BMI160 INT1 for interrupt-driven collection |
| `update_interval` | time | 5s | How often to perform FFT analysis |

#### `sensor` Platform

| Sensor | Unit | Description |
|--------|------|-------------|
| `peak_frequency` | Hz | Dominant vibration frequency |
| `peak_magnitude` | g | Magnitude of the peak frequency component |
| `total_energy` | g | RMS energy across all frequency bins |
| `dominant_frequency_energy` | g | Energy concentrated around the peak frequency |
| `rpm` | RPM | Peak frequency converted to rotations per minute |

#### `binary_sensor` Platform

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `running` | binary_sensor | - | Indicates when vibration energy exceeds threshold |
| `energy_threshold` | float | 0.05 | Energy level (in g) that triggers the running state |
| `timeout` | time | 60s | How long to wait after vibration stops before turning off |

#### `number` Platform (Optional)

Expose configurable parameters as Home Assistant number entities:

```yaml
number:
  - platform: bmi160_fft
    energy_threshold:
      name: "Vibration Energy Threshold"
    running_timeout:
      name: "Running Timeout"
    frequency_threshold:
      name: "Frequency Threshold"
```

| Number | Range | Description |
|--------|-------|-------------|
| `energy_threshold` | 0.001 - 1.0 | Running detection threshold (g) |
| `running_timeout` | 1 - 300 | Timeout in seconds |
| `frequency_threshold` | 0 - 50 | Minimum frequency to report (Hz), filters out noise |

## Interrupt vs Polling Mode

### Polling Mode (Default)
When no interrupt pin is configured, the component reads accelerometer data at precise intervals during each update cycle. This is simpler but blocks the CPU during sample collection.

### Interrupt Mode (Recommended)
When an interrupt pin is configured, the component uses the BMI160's FIFO and watermark interrupt for efficient data collection. The CPU is free to do other tasks while samples accumulate in the sensor's hardware FIFO.

```yaml
bmi160_fft:
  cs_pin: GPIO5
  interrupt_pin: GPIO4  # Enable interrupt mode
```

## Use Cases

### Washing Machine Monitoring

Detect when your washing machine is running and get notifications when the cycle completes:

```yaml
binary_sensor:
  - platform: bmi160_fft
    running:
      name: "Washing Machine Running"
      energy_threshold: 0.03
      timeout: 120s  # 2 minutes after vibration stops
```

### Motor RPM Monitoring

Monitor the speed of rotating machinery:

```yaml
sensor:
  - platform: bmi160_fft
    peak_frequency:
      name: "Motor Frequency"
    rpm:
      name: "Motor RPM"
      filters:
        - sliding_window_moving_average:
            window_size: 5
```

## Frequency Resolution

The frequency resolution depends on the FFT size and sample rate:

```
Resolution = Sample Rate / FFT Size
```

| FFT Size | @ 1600 Hz | @ 800 Hz |
|----------|-----------|----------|
| 256      | 6.25 Hz   | 3.13 Hz  |
| 512      | 3.13 Hz   | 1.56 Hz  |
| 1024     | 1.56 Hz   | 0.78 Hz  |
| 2048     | 0.78 Hz   | 0.39 Hz  |

Higher FFT sizes provide better frequency resolution but require more memory and processing time.

## Troubleshooting

### "Wrong chip ID" Error
- Verify SPI wiring connections
- Ensure the BMI160 is powered with 3.3V
- Check that CS, MISO, MOSI, and CLK pins are correctly connected

### No Vibration Detected
- Lower the `energy_threshold` value
- Ensure the sensor is firmly mounted to the vibrating surface
- Check that the accelerometer is in normal mode (visible in logs)

### Erratic Readings
- Increase `fft_size` for better frequency resolution
- Add filtering to sensors
- Ensure stable power supply to the sensor

## License

MIT License
