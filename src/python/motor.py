#! /usr/bin/env python3
import pigpio
import os
import logging
import sys

logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)

class Motor:
    '''A brushless DC motor driven by one pin on a Raspberry Pi.'''

    def __init__ (self, pin, frequency=50):
        #  Configure GPIO
        if os.system ("pgrep pigpiod") != 0:
            logging.debug("Starting pipgiod")
            os.system("sudo pigpiod")

        self._pi = pigpio.pi()  # Connect to PIGPIO-Daemon
        self._pin = pin
        self._frequency = frequency
        self._speed = 0

        self._pi.set_mode(pin, pigpio.OUTPUT)
        self._pi.set_PWM_frequency(pin, frequency)
        self._pi.set_PWM_range(pin, 100)  # Control PWM from 0 .. 100 %

    def speedperc2dc(self, mot_speed_perc):
        min_dc = 5      # Corresponds to 1 ms @ 50 Hz
        max_dc = 10     # Corresponds to 2 ms @ 50 Hz
        dc = min(max(7.5 + mot_speed_perc*2.5/100, min_dc), max_dc)
        print('DutyCycle: ' + str(dc))
        return dc

    def arm(self):
        self.set_motor_speed(0)

    def set_motor_speed (self, speed):      
        self._pi.set_PWM_dutycycle(self._pin, self.speedperc2dc(speed))
        self._speed = speed
