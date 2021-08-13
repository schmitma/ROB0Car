#!/usr/bin/python3
from motor import Motor
from steering import Steering
import time
import logging
import sys

logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)

class ROB0Car:
    def __init__ (self):
        logging.debug("ROBOCar.__init__")
        self.leftMotor = Motor(4, 50)
        self.rightMotor = Motor(5, 50)
        self.steering = Steering(3)

    def drive(self, speed):
        logging.debug("ROBOCar.drive")
        self.leftMotor.set_motor_speed(speed)
        self.rightMotor.set_motor_speed(speed)

    def steer(self, steering_dc):
        self.set

def main():
    rob0car = ROB0Car()
    
    for s in range(1000,2000,50):
        rob0car.steering.set_steering_dc(s)
        time.sleep(1)

    for s in range(0,100,10):
        rob0car.drive(s)
        time.sleep(1)

    for s in range(100,0,-10):
        rob0car.drive(s)
        time.sleep(1)

if __name__ == '__main__':
    main()