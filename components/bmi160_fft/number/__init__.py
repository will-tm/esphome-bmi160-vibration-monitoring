import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.const import (
    CONF_ID,
    CONF_MODE,
    ENTITY_CATEGORY_CONFIG,
)
from .. import bmi160_fft_ns, BMI160FFTComponent, CONF_BMI160_FFT_ID

DEPENDENCIES = ['bmi160_fft']

CONF_ENERGY_THRESHOLD = 'energy_threshold'
CONF_RUNNING_TIMEOUT = 'running_timeout'
CONF_FREQUENCY_MIN = 'frequency_min'
CONF_FREQUENCY_MAX = 'frequency_max'

BMI160FFTNumber = bmi160_fft_ns.class_('BMI160FFTNumber', number.Number, cg.Component)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(CONF_BMI160_FFT_ID): cv.use_id(BMI160FFTComponent),
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
    cv.Optional(CONF_FREQUENCY_MIN): number.number_schema(
        BMI160FFTNumber,
        entity_category=ENTITY_CATEGORY_CONFIG,
        icon="mdi:sine-wave",
    ),
    cv.Optional(CONF_FREQUENCY_MAX): number.number_schema(
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
        await cg.register_component(n, config[CONF_ENERGY_THRESHOLD])
        cg.add(n.set_parent(parent))
        cg.add(n.set_number_type(0))  # 0 = energy threshold
        cg.add(parent.set_energy_threshold_number(n))

    if CONF_RUNNING_TIMEOUT in config:
        n = await number.new_number(
            config[CONF_RUNNING_TIMEOUT],
            min_value=1,
            max_value=300,
            step=1,
        )
        await cg.register_component(n, config[CONF_RUNNING_TIMEOUT])
        cg.add(n.set_parent(parent))
        cg.add(n.set_number_type(1))  # 1 = timeout
        cg.add(parent.set_timeout_number(n))

    if CONF_FREQUENCY_MIN in config:
        n = await number.new_number(
            config[CONF_FREQUENCY_MIN],
            min_value=0.0,
            max_value=800.0,
            step=0.1,
        )
        await cg.register_component(n, config[CONF_FREQUENCY_MIN])
        cg.add(n.set_parent(parent))
        cg.add(n.set_number_type(2))  # 2 = frequency min
        cg.add(parent.set_frequency_min_number(n))

    if CONF_FREQUENCY_MAX in config:
        n = await number.new_number(
            config[CONF_FREQUENCY_MAX],
            min_value=0.0,
            max_value=800.0,
            step=0.1,
        )
        await cg.register_component(n, config[CONF_FREQUENCY_MAX])
        cg.add(n.set_parent(parent))
        cg.add(n.set_number_type(3))  # 3 = frequency max
        cg.add(parent.set_frequency_max_number(n))
