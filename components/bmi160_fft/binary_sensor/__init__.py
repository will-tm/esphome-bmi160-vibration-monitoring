import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import CONF_ID
from .. import bmi160_fft_ns, BMI160FFT, CONF_BMI160_FFT_ID

DEPENDENCIES = ['bmi160_fft']

CONF_RUNNING = 'running'
CONF_ENERGY_THRESHOLD = 'energy_threshold'
CONF_TIMEOUT = 'timeout'

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(CONF_BMI160_FFT_ID): cv.use_id(BMI160FFT),
    cv.Optional(CONF_RUNNING): binary_sensor.binary_sensor_schema(
        device_class="running",
        icon="mdi:washing-machine",
    ).extend({
        cv.Optional(CONF_ENERGY_THRESHOLD, default=0.05): cv.float_range(min=0.001, max=10.0),
        cv.Optional(CONF_TIMEOUT, default="60s"): cv.positive_time_period_seconds,
    }),
})


async def to_code(config):
    parent = await cg.get_variable(config[CONF_BMI160_FFT_ID])

    if CONF_RUNNING in config:
        running_config = config[CONF_RUNNING]
        sens = await binary_sensor.new_binary_sensor(running_config)
        cg.add(parent.set_running_binary_sensor(sens))
        cg.add(parent.set_running_threshold(running_config[CONF_ENERGY_THRESHOLD]))
        cg.add(parent.set_running_timeout(running_config[CONF_TIMEOUT].total_seconds))
