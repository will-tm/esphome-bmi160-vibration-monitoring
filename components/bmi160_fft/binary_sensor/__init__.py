import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import CONF_ID
from .. import bmi160_fft_ns, BMI160FFT, CONF_BMI160_FFT_ID

DEPENDENCIES = ['bmi160_fft']

CONF_RUNNING = 'running'

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(CONF_BMI160_FFT_ID): cv.use_id(BMI160FFT),
    cv.Optional(CONF_RUNNING): binary_sensor.binary_sensor_schema(
        device_class="running",
        icon="mdi:washing-machine",
    ),
})


async def to_code(config):
    parent = await cg.get_variable(config[CONF_BMI160_FFT_ID])

    if CONF_RUNNING in config:
        running_config = config[CONF_RUNNING]
        sens = await binary_sensor.new_binary_sensor(running_config)
        cg.add(parent.set_running_binary_sensor(sens))
