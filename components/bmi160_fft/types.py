import esphome.codegen as cg
from esphome.components import spi

bmi160_fft_ns = cg.esphome_ns.namespace('bmi160_fft')
BMI160FFT = bmi160_fft_ns.class_('BMI160FFT', cg.PollingComponent, spi.SPIDevice)
