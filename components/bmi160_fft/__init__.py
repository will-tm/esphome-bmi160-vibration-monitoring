import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID
from esphome.components import spi
from esphome import pins
from .types import bmi160_fft_ns, BMI160FFT

DEPENDENCIES = ['spi']
AUTO_LOAD = ['sensor', 'binary_sensor', 'number']

CONF_BMI160_FFT_ID = 'bmi160_fft_id'
CONF_FFT_SIZE = 'fft_size'
CONF_SAMPLE_RATE = 'sample_rate'
CONF_ACCEL_RANGE = 'accel_range'
CONF_INTERRUPT_PIN = 'interrupt_pin'

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(BMI160FFT),
    cv.Optional(CONF_FFT_SIZE, default=1024): cv.one_of(128, 256, 512, 1024, 2048, 4096, int=True),
    cv.Optional(CONF_SAMPLE_RATE, default=1600): cv.one_of(100, 200, 400, 800, 1600, int=True),
    cv.Optional(CONF_ACCEL_RANGE, default=4): cv.one_of(2, 4, 8, 16, int=True),
    cv.Optional(CONF_INTERRUPT_PIN): pins.internal_gpio_input_pin_schema,
}).extend(cv.polling_component_schema('5s')).extend(spi.spi_device_schema(cs_pin_required=True))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await spi.register_spi_device(var, config)

    cg.add(var.set_fft_size(config[CONF_FFT_SIZE]))
    cg.add(var.set_sample_rate(config[CONF_SAMPLE_RATE]))
    cg.add(var.set_accel_range(config[CONF_ACCEL_RANGE]))

    if CONF_INTERRUPT_PIN in config:
        pin = await cg.gpio_pin_expression(config[CONF_INTERRUPT_PIN])
        cg.add(var.set_interrupt_pin(pin))
