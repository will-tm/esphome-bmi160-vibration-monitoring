import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.const import (
    CONF_ID,
    CONF_MODE,
    ENTITY_CATEGORY_CONFIG,
)
from .. import bmi160_fft_ns, BMI160FFT, CONF_BMI160_FFT_ID

DEPENDENCIES = ['bmi160_fft']

CONF_ENERGY_THRESHOLD = 'energy_threshold'
CONF_RUNNING_TIMEOUT = 'running_timeout'
CONF_FREQUENCY_THRESHOLD = 'frequency_threshold'

BMI160FFTNumber = bmi160_fft_ns.class_('BMI160FFTNumber', number.Number, cg.Component)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(CONF_BMI160_FFT_ID): cv.use_id(BMI160FFT),
    cv.Optional(CONF_ENERGY_THRESHOLD): number.number_schema(
        BMI160FFTNumber,
        entity_category=ENTITY_CATEGORY_CONFIG,
        icon="mdi:gauge",
    ),
    cv.Optional(CONF_RUNNING_TIMEOUT): number.number_schema(
        BMI160FFTNumber,
        entity_category=ENTITY_CATEGORY_CONFIG,
        icon="mdi:timer-outline",
    ),
    cv.Optional(CONF_FREQUENCY_THRESHOLD): number.number_schema(
        BMI160FFTNumber,
        entity_category=ENTITY_CATEGORY_CONFIG,
        icon="mdi:sine-wave",
    ),
})


async def to_code(config):
    parent = await cg.get_variable(config[CONF_BMI160_FFT_ID])

    if CONF_ENERGY_THRESHOLD in config:
        n = await number.new_number(
            config[CONF_ENERGY_THRESHOLD],
            min_value=0.001,
            max_value=1.0,
            step=0.001,
        )
        cg.add(n.set_parent(parent))
        cg.add(n.set_type(0))  # 0 = energy threshold
        cg.add(parent.set_energy_threshold_number(n))

    if CONF_RUNNING_TIMEOUT in config:
        n = await number.new_number(
            config[CONF_RUNNING_TIMEOUT],
            min_value=1,
            max_value=300,
            step=1,
        )
        cg.add(n.set_parent(parent))
        cg.add(n.set_type(1))  # 1 = timeout
        cg.add(parent.set_timeout_number(n))

    if CONF_FREQUENCY_THRESHOLD in config:
        n = await number.new_number(
            config[CONF_FREQUENCY_THRESHOLD],
            min_value=0.0,
            max_value=50.0,
            step=0.1,
        )
        cg.add(n.set_parent(parent))
        cg.add(n.set_type(2))  # 2 = frequency threshold
        cg.add(parent.set_frequency_threshold_number(n))
