import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    UNIT_HERTZ,
    DEVICE_CLASS_FREQUENCY,
    STATE_CLASS_MEASUREMENT,
)
from .. import bmi160_fft_ns, BMI160FFTComponent, CONF_BMI160_FFT_ID

DEPENDENCIES = ['bmi160_fft']

CONF_PEAK_FREQUENCY = 'peak_frequency'
CONF_PEAK_MAGNITUDE = 'peak_magnitude'
CONF_TOTAL_ENERGY = 'total_energy'
CONF_DOMINANT_FREQUENCY_ENERGY = 'dominant_frequency_energy'
CONF_RPM = 'rpm'

UNIT_G = 'g'
UNIT_RPM = 'RPM'
ICON_VIBRATE = 'mdi:vibrate'
ICON_RPM = 'mdi:rotate-3d-variant'

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
