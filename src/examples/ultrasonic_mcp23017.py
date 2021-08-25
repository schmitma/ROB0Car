#!/usr/bin/python3
import RPi.GPIO as GPIO
import time, sys, logging, smbus

logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)

bus = smbus.SMBus(1)
mcp23017_i2c_addr = 0x27

IODIRA = 0x00
IODIRB = 0x01
GPIOB = 0x13
OLATA = 0x14

ULTRASONIC_FRONT_LEFT = 0x01

def get(register):
    read = bus.read_byte_data(mcp23017_i2c_addr, register)
    return read

def set(register, data):
    write = bus.write_byte_data(mcp23017_i2c_addr, register, data)
    return write

# Configure pin bank A as outputs for triggering
set(IODIRA, 0x00)
set(OLATA, 0x00)
# Configure pin bank B as inputs for receiving echo
set(IODIRB, 0xFF)

def meas_dist(sensor_flag):
    logging.debug("meas_dist()")

    set(OLATA, sensor_flag)
    time.sleep(0.00001) # 10 us
    set(OLATA, 0x00)
    logging.debug("meas_dist()")

    state = get(GPIOB)
    logging.debug("Waiting for echo pin to turn HIGH")
    while (state & sensor_flag) == 0x00:
        logging.debug(str(state))
        state = get(GPIOB)
        pass
    start = time.time()
    
    state = get(GPIOB)
    logging.debug("Waiting for echo pin to turn LOW")
    while (state & GPIOB) == GPIOB:
        logging.debug(str(state))
        state = get(GPIOB)
        pass
    end = time.time()

    distance = ((end - start) * 34300) / 2
    print("Distance: ", distance, " cm")
    return distance

while True:
    meas_dist(ULTRASONIC_FRONT_LEFT)
    time.sleep(0.5)
