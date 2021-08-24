#!/usr/bin/python3
import RPi.GPIO as GPIO
import time, sys, logging, smbus

logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)

bus = smbus.SMBus(1)
mcp23017_i2c_addr = 0x27

def get(register):
    read = bus.read_byte_data(mcp23017_i2c_addr, register)
    return read

def set(register, data):
    write = bus.write_byte_data(mcp23017_i2c_addr, register, data)
    return write

set(0x00, 0x20)

while True:
    state = get(0x12)
    print(state)
    if state == 0x00:
        set(0x14, 0x01)
        print("Button pressed --> LED on")
        time.sleep(1)
        set(0x14, 0x00)
