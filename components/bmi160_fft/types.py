import esphome.codegen as cg
from esphome.components import spi

bmi160_fft_ns = cg.esphome_ns.namespace('bmi160_fft')
BMI160FFTComponent = bmi160_fft_ns.class_('BMI160FFTComponent', cg.PollingComponent)
BMI160Driver = bmi160_fft_ns.class_('BMI160Driver', spi.SPIDevice)
