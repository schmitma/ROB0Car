#!/usr/bin/python3
import RPi.GPIO as GPIO
import time, sys, logging, smbus

logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)

bus = smbus.SMBus(1)
pcf8574_i2c_addr = 0x27

P0 = 0x01
P1 = 0x02
P2 = 0x04
P3 = 0x08
P4 = 0x10
P5 = 0x20
P6 = 0x40
P7 = 0x80

while True:
    b = bus.read_byte(pcf8574_i2c_addr, P0)
    logging.debug(str(b))
    time.sleep(0.5)
    bus.write_byte(pcf8574_i2c_addr, P1)
    time.sleep(0.5)
    bus.write_byte(pcf8574_i2c_addr, P2)
    time.sleep(0.5)
    bus.write_byte(pcf8574_i2c_addr, P3)
    time.sleep(0.5)
    bus.write_byte(pcf8574_i2c_addr, P4)
    time.sleep(0.5)
    bus.write_byte(pcf8574_i2c_addr, P5)
    time.sleep(0.5)
    bus.write_byte(pcf8574_i2c_addr, P6)
    time.sleep(0.5)
    bus.write_byte(pcf8574_i2c_addr, P7)
    time.sleep(0.5)
