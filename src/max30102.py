# -*- coding: utf-8 -*-
from __future__ import print_function
from time import sleep
import smbus2 as smbus
import RPi.GPIO as GPIO

REG_INTR_STATUS_1   = 0x00
REG_INTR_STATUS_2   = 0x01
REG_INTR_ENABLE_1   = 0x02
REG_INTR_ENABLE_2   = 0x03
REG_FIFO_WR_PTR     = 0x04
REG_OVF_COUNTER     = 0x05
REG_FIFO_RD_PTR     = 0x06
REG_FIFO_DATA       = 0x07
REG_FIFO_CONFIG     = 0x08
REG_MODE_CONFIG     = 0x09
REG_SPO2_CONFIG     = 0x0A
REG_LED1_PA         = 0x0C
REG_LED2_PA         = 0x0D
REG_PILOT_PA        = 0x10
REG_TEMP_INTR       = 0x1F
REG_TEMP_FRAC       = 0x20
REG_TEMP_CONFIG     = 0x21
REG_PROX_INT_THRESH = 0x30
REG_REV_ID          = 0xFE
REG_PART_ID         = 0xFF

class MAX30102(object):
    """
    Minimal driver for MAX30102
    - Default I2C: bus 1, address 0x57
    - Optional GPIO interrupt; pass gpio_pin=None for polling-only
    """
    def __init__(self, channel=1, address=0x57, gpio_pin=7):
        print("Channel: {0}, address: 0x{1:x}".format(channel, address))
        self.address = address
        self.channel = channel
        self.bus = smbus.SMBus(self.channel)
        self.interrupt = gpio_pin

        if self.interrupt is not None:
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(self.interrupt, GPIO.IN)
        else:
            print("No INT pin configured; will poll FIFO without waiting for GPIO")

        self.reset()
        sleep(0.8)

        self.bus.read_i2c_block_data(self.address, REG_INTR_STATUS_1, 1)
        self.setup()

    def shutdown(self):
        self.bus.write_i2c_block_data(self.address, REG_MODE_CONFIG, [0x80])

    def reset(self):
        self.bus.write_i2c_block_data(self.address, REG_MODE_CONFIG, [0x40])

    def setup(self, led_mode=0x03):
        # Interrupts: A_FULL_EN + PPG_RDY_EN
        self.bus.write_i2c_block_data(self.address, REG_INTR_ENABLE_1, [0xC0])
        self.bus.write_i2c_block_data(self.address, REG_INTR_ENABLE_2, [0x00])

        # FIFO pointers
        self.bus.write_i2c_block_data(self.address, REG_FIFO_WR_PTR, [0x00])
        self.bus.write_i2c_block_data(self.address, REG_OVF_COUNTER, [0x00])
        self.bus.write_i2c_block_data(self.address, REG_FIFO_RD_PTR, [0x00])

        # FIFO config: sample avg=4, rollover=0, almost full=17
        self.bus.write_i2c_block_data(self.address, REG_FIFO_CONFIG, [0x4F])

        # Mode: SpO2
        self.bus.write_i2c_block_data(self.address, REG_MODE_CONFIG, [led_mode])

        # SPO2: ADC range=4096nA, sample rate=100Hz, pulse-width=411us
        self.bus.write_i2c_block_data(self.address, REG_SPO2_CONFIG, [0x27])

        # LED currents
        self.bus.write_i2c_block_data(self.address, REG_LED1_PA, [0x24])  # ~7mA
        self.bus.write_i2c_block_data(self.address, REG_LED2_PA, [0x24])  # ~7mA
        self.bus.write_i2c_block_data(self.address, REG_PILOT_PA, [0x7F]) # ~25mA

    def set_config(self, reg, value):
        self.bus.write_i2c_block_data(self.address, reg, value)

    def flush_fifo(self):
        # reset pointers to drop unread samples
        self.bus.write_i2c_block_data(self.address, REG_FIFO_WR_PTR, [0x00])
        self.bus.write_i2c_block_data(self.address, REG_OVF_COUNTER, [0x00])
        self.bus.write_i2c_block_data(self.address, REG_FIFO_RD_PTR, [0x00])
        # read/clear interrupt
        self.bus.read_i2c_block_data(self.address, REG_INTR_STATUS_1, 1)
        self.bus.read_i2c_block_data(self.address, REG_INTR_STATUS_2, 1)

    def _read_fifo_pair(self):
        d = self.bus.read_i2c_block_data(self.address, REG_FIFO_DATA, 6)
        red = (d[0] << 16 | d[1] << 8 | d[2]) & 0x03FFFF
        ir  = (d[3] << 16 | d[4] << 8 | d[5]) & 0x03FFFF
        return red, ir

    def read_sequential(self, amount=100):
        red_buf, ir_buf = [], []
        for _ in range(amount):
            if self.interrupt is not None:
                while GPIO.input(self.interrupt) == 1:
                    pass
            # read status (clears flags)
            self.bus.read_i2c_block_data(self.address, REG_INTR_STATUS_1, 1)
            self.bus.read_i2c_block_data(self.address, REG_INTR_STATUS_2, 1)
            r, i = self._read_fifo_pair()
            red_buf.append(r)
            ir_buf.append(i)
        return red_buf, ir_buf
