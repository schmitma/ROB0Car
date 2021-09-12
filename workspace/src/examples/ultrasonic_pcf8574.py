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

trig = P0
echo = P4

bus.write_byte(pcf8574_i2c_addr, 0x00)

# GPIO.setmode(GPIO.BOARD)
# trig = [11] #, 36]
# echo = [13] #, 38]

# for i in range(len(trig)):
#     GPIO.setup(echo[i], GPIO.IN)
#     GPIO.setup(trig[i], GPIO.OUT)

def meas_dist():
    logging.debug("meas_dist()")

    # print("Trigger Pin: ", trig[n], 
    #   "\nEcho Pin: ", echo[n])
    # GPIO.output(trig[n], True)
    # time.sleep(0.00001) # 10 Mikrosekunden
    # GPIO.output(trig[n], False)

    bus.write_byte(pcf8574_i2c_addr, trig)
    time.sleep(0.00001) # 10 us
    bus.write_byte(pcf8574_i2c_addr, 0x00)
    logging.debug("meas_dist()")

    b = bus.read_byte(pcf8574_i2c_addr)
    logging.debug("Waiting for echo pin to turn HIGH")
    while (b & echo) == 0x00:
        logging.debug(str(b))
        b = bus.read_byte(pcf8574_i2c_addr)
        pass
    start = time.time()
    
    b = bus.read_byte(pcf8574_i2c_addr, trig)
    logging.debug("Waiting for echo pin to turn LOW")
    while (b & echo) == echo:
        logging.debug(str(b))
        b = bus.read_byte(pcf8574_i2c_addr, trig)
        pass
    end = time.time()

    distance = ((end - start) * 34300) / 2
    print("Distance: ", distance, " cm")
    return distance

while True:
    meas_dist()
    time.sleep(0.5)
