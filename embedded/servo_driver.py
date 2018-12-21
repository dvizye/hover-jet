#!/usr/bin/python
import smbus2
import time
import RPi.GPIO as GPIO

class Servo:
    I2C_ADDRESS = 0x40 
    MODE1_ADDR = 0x00
    MODE2_ADDR = 0x01
    PRESCALER_ADDR = 0xFE
    OSC_CLOCK = 25e6
    CH0_OFFSET = 0x6
    CNT_MAX = 4096

    def init(self, i2cbus, oe_pin):
        self.bus = smbus2.SMBus(i2cbus)
        self.oe_pin = oe_pin
        self.enable_autoinc()
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.oe_pin, GPIO.OUT)
        self.enable_output(False)

    def enable_autoinc(self):
        self.set_register_bit(self.MODE1_ADDR, 5)

    def enable_output(self, enable):
        if (enable):
            GPIO.output(self.oe_pin, GPIO.LOW)
        else:
            GPIO.output(self.oe_pin, GPIO.HIGH)

    def read_byte(self, register):
        return self.bus.read_byte_data(self.I2C_ADDRESS, register)

    def read_word(self, register):
        return self.bus.read_word_data(self.I2C_ADDRESS, register)

    def write_byte(self, register, value):
        self.bus.write_byte_data(self.I2C_ADDRESS, register, value)

    def write_word(self, register, value):
        self.bus.write_word_data(self.I2C_ADDRESS, register, value)

    def set_register_bit(self, register, bit):
        current = self.read_byte(register)
        new = (1 << bit) | current
        self.write_byte(register, new)

    def clear_register_bit(self, register, bit):
        current = self.read_byte(register)
        new = ~(1 << bit) & current
        self.write_byte(register, new)

    def turn_on(self):
        current_mode = self.read_byte(self.MODE1_ADDR)
        if (current_mode & (1 << 7) != 0):
            self.clear_register_bit(self.MODE1_ADDR, 4)
            time.sleep(0.001)
        self.set_register_bit(self.MODE1_ADDR, 7)

    def disable(self):
        self.set_register_bit(self.MODE1_ADDR, 4)

    def enable(self):
        self.clear_register_bit(self.MODE1_ADDR, 4)

    def set_update_rate_hz(self, update_freq_hz):
        self.disable()
        prescaler = (int(self.OSC_CLOCK/(self.CNT_MAX * update_freq_hz))) - 1
        self.write_byte(self.PRESCALER_ADDR, prescaler)
        self.enable()

    def set_update_rate_us(self, update_period_us):
        self.set_update_rate_hz(1.0/update_period_us)

    def set_duty(self, channel, duty_percent, offset_percent = 0):
        register_offset = (channel * 4) + self.CH0_OFFSET
        if duty_percent == 100:
            self.set_register_bit(register_offset + 1, 4)
            self.clear_register_bit(register_offest + 3, 4)
        elif duty_percent == 0:
            self.set_register_bit(register_offset + 3, 4)
            self.clear_register_bit(register_offset + 1, 4)
        else:
            self.clear_register_bit(register_offset + 3, 4)
            self.clear_register_bit(register_offset + 1, 4)
            if offset_percent == 0:
                delay_cnt = 0
            else:
                delay_cnt = int((offset_percent/100.0) * self.CNT_MAX) - 1
            off_cnt = int((duty_percent/100.0) *  self.CNT_MAX) - 1 + delay_cnt
            self.write_word(register_offset, delay_cnt)
            self.write_word(register_offset + 2, off_cnt)
            

servo = Servo()
servo.init(1, 14)

servo.set_update_rate_us(16666)
servo.set_duty(0, 0)

servo.enable_output(True)


