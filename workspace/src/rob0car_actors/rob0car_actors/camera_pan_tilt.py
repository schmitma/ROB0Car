#!/usr/bin/python3
import logging
import sys
from rob0car_actors.pca9685 import PCA9685

logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)

PAN_CHANNEL = 0
TILT_CHANNEL = 1

MIN_TILT_ANGLE = 0
MAX_TILT_ANGLE = 180

MIN_PAN_ANGLE = 0
MAX_PAN_ANGLE = 180

class CameraPanTilt:
    def __init__ (self):
        logging.debug("CameraPanTilt.__init__()")

        self.pca9685 = PCA9685()
        self.pca9685.setPWMFreq(50)

        self.pan_angle = 90
        self.tilt_angle = 90

        self.home

    def pan(self, pan_angle):
        logging.debug("CameraPanTilt.pan()")
        self.pan_angle = min(max(MIN_PAN_ANGLE, pan_angle), MAX_PAN_ANGLE)
        self.pca9685.setRotationAngle(PAN_CHANNEL, self.pan_angle)

    def tilt(self, tilt_angle):
        logging.debug("CameraPanTilt.tilt()")
        self.tilt_angle = min(max(MIN_TILT_ANGLE, tilt_angle), MAX_TILT_ANGLE)
        self.pca9685.setRotationAngle(TILT_CHANNEL, self.tilt_angle)

    def home(self):
        logging.debug("CameraPanTilt.home()")
        self.pan_angle = 90
        self.tilt_angle = 90

        self.pca9685.setRotationAngle(PAN_CHANNEL, self.tilt_angle)
        self.pca9685.setRotationAngle(TILT_CHANNEL, self.pan_angle)