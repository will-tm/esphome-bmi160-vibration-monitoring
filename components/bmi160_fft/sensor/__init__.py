import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    CONF_COUNT,
    UNIT_HERTZ,
    DEVICE_CLASS_FREQUENCY,
    STATE_CLASS_MEASUREMENT,
)
from esphome.core import CORE
from .. import bmi160_fft_ns, BMI160FFTComponent, CONF_BMI160_FFT_ID

DEPENDENCIES = ['bmi160_fft']

CONF_PEAK_FREQUENCY = 'peak_frequency'
CONF_PEAK_MAGNITUDE = 'peak_magnitude'
CONF_TOTAL_ENERGY = 'total_energy'
CONF_DOMINANT_FREQUENCY_ENERGY = 'dominant_frequency_energy'
CONF_RPM = 'rpm'
CONF_PEAKS = 'peaks'
CONF_FREQUENCY_NAME = 'frequency_name'
CONF_MAGNITUDE_NAME = 'magnitude_name'

UNIT_G = 'g'
UNIT_RPM = 'RPM'
ICON_VIBRATE = 'mdi:vibrate'
ICON_RPM = 'mdi:rotate-3d-variant'

MAX_PEAKS = 8

PEAKS_SCHEMA = cv.Schema({
    cv.Required(CONF_COUNT): cv.int_range(min=0, max=MAX_PEAKS),
    cv.Optional(CONF_FREQUENCY_NAME, default="Peak {} Frequency"): cv.string,
    cv.Optional(CONF_MAGNITUDE_NAME, default="Peak {} Magnitude"): cv.string,
})

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(CONF_BMI160_FFT_ID): cv.use_id(BMI160FFTComponent),
    cv.Optional(CONF_PEAK_FREQUENCY): sensor.sensor_schema(
        unit_of_measurement=UNIT_HERTZ,
        accuracy_decimals=2,
        device_class=DEVICE_CLASS_FREQUENCY,
        state_class=STATE_CLASS_MEASUREMENT,
        icon="mdi:sine-wave",
    ),
    cv.Optional(CONF_PEAK_MAGNITUDE): sensor.sensor_schema(
        unit_of_measurement=UNIT_G,
        accuracy_decimals=4,
        state_class=STATE_CLASS_MEASUREMENT,
        icon=ICON_VIBRATE,
    ),
    cv.Optional(CONF_TOTAL_ENERGY): sensor.sensor_schema(
        unit_of_measurement=UNIT_G,
        accuracy_decimals=4,
        state_class=STATE_CLASS_MEASUREMENT,
        icon=ICON_VIBRATE,
    ),
    cv.Optional(CONF_DOMINANT_FREQUENCY_ENERGY): sensor.sensor_schema(
        unit_of_measurement=UNIT_G,
        accuracy_decimals=4,
        state_class=STATE_CLASS_MEASUREMENT,
        icon=ICON_VIBRATE,
    ),
    cv.Optional(CONF_RPM): sensor.sensor_schema(
        unit_of_measurement=UNIT_RPM,
        accuracy_decimals=1,
        state_class=STATE_CLASS_MEASUREMENT,
        icon=ICON_RPM,
    ),
    cv.Optional(CONF_PEAKS): PEAKS_SCHEMA,
})


async def to_code(config):
    parent = await cg.get_variable(config[CONF_BMI160_FFT_ID])

    if CONF_PEAK_FREQUENCY in config:
        sens = await sensor.new_sensor(config[CONF_PEAK_FREQUENCY])
        cg.add(parent.set_peak_frequency_sensor(sens))

    if CONF_PEAK_MAGNITUDE in config:
        sens = await sensor.new_sensor(config[CONF_PEAK_MAGNITUDE])
        cg.add(parent.set_peak_magnitude_sensor(sens))

    if CONF_TOTAL_ENERGY in config:
        sens = await sensor.new_sensor(config[CONF_TOTAL_ENERGY])
        cg.add(parent.set_total_energy_sensor(sens))

    if CONF_DOMINANT_FREQUENCY_ENERGY in config:
        sens = await sensor.new_sensor(config[CONF_DOMINANT_FREQUENCY_ENERGY])
        cg.add(parent.set_dominant_frequency_energy_sensor(sens))

    if CONF_RPM in config:
        sens = await sensor.new_sensor(config[CONF_RPM])
        cg.add(parent.set_rpm_sensor(sens))

    # Process peaks - only when MQTT is NOT configured (they conflict)
    if CONF_PEAKS in config:
        # Check if MQTT is enabled
        mqtt_enabled = "mqtt" in CORE.config

        peaks_config = config[CONF_PEAKS]
        count = peaks_config[CONF_COUNT]
        freq_name_template = peaks_config[CONF_FREQUENCY_NAME]
        mag_name_template = peaks_config[CONF_MAGNITUDE_NAME]

        if count > 0:
            if mqtt_enabled:
                # Skip peaks when MQTT is enabled - they conflict
                import esphome.core as core
                core._LOGGER.warning(
                    "Peak sensors are disabled when MQTT is enabled. "
                    "Remove mqtt: from config to enable peak sensors."
                )
            else:
                cg.add(parent.set_num_peaks(count))

                for i in range(count):
                    # Build frequency sensor config through schema with unique ID
                    freq_schema = sensor.sensor_schema(
                        unit_of_measurement=UNIT_HERTZ,
                        accuracy_decimals=2,
                        device_class=DEVICE_CLASS_FREQUENCY,
                        state_class=STATE_CLASS_MEASUREMENT,
                        icon="mdi:sine-wave",
                    )
                    freq_config = freq_schema({
                        CONF_ID: f"bmi160_peak_{i+1}_frequency",
                        "name": freq_name_template.format(i + 1),
                    })
                    freq_sens = await sensor.new_sensor(freq_config)
                    cg.add(parent.set_peak_frequency_sensor(i, freq_sens))

                    # Build magnitude sensor config through schema with unique ID
                    mag_schema = sensor.sensor_schema(
                        unit_of_measurement=UNIT_G,
                        accuracy_decimals=4,
                        state_class=STATE_CLASS_MEASUREMENT,
                        icon=ICON_VIBRATE,
                    )
                    mag_config = mag_schema({
                        CONF_ID: f"bmi160_peak_{i+1}_magnitude",
                        "name": mag_name_template.format(i + 1),
                    })
                    mag_sens = await sensor.new_sensor(mag_config)
                    cg.add(parent.set_peak_magnitude_sensor(i, mag_sens))
