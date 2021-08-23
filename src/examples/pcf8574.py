#!/usr/bin/python3
import RPi.GPIO as GPIO
import time, sys, logging, smbus

logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)

bus = smbus.SMBus(1)
pcf8574_i2c_addr = 0x27

P0 = 0x01
P1 = 0x02

GPIO.setmode(GPIO.BOARD)
trig = [11] #, 36]
echo = [13] #, 38]

for i in range(len(trig)):
    GPIO.setup(echo[i], GPIO.IN)
    GPIO.setup(trig[i], GPIO.OUT)

def meas_dist(n):
    logging.debug("meas_dist()")

    # print("Trigger Pin: ", trig[n], 
    #   "\nEcho Pin: ", echo[n])
    GPIO.output(trig[n], True)
    time.sleep(0.00001) # 10 Mikrosekunden
    GPIO.output(trig[n], False)
    b = bus.read_byte_data(pcf8574_i2c_addr)

    while (b & P0):
        pass
    start = time.time()

    while not (b & P0):
        pass
    end = time.time()

    distance = ((end - start) * 34300) / 2
    print("Distance ", n, ": ", distance, " cm")
    return distance

while True:
    bus.write_byte(pcf8574_i2c_addr, P0)

    time.sleep(0.5)
    bus.write_byte(pcf8574_i2c_addr, P1)
    time.sleep(0.5)
