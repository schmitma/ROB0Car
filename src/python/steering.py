#!/usr/bin/python3
import os
import pigpio

class Steering:
    '''A servo motor driven by one pin on a Raspberry Pi.'''

    def __init__ (self, pin):
        #  Configure GPIO
        if os.system ("pgrep pigpiod") != 0:
            os.system("sudo pigpiod")

        self._pi = pigpio.pi()  # Connect to PIGPIO-Daemon
        self._pin = pin

        self._pi.set_mode(pin, pigpio.OUTPUT)
        self._pi.set_servo_pulsewidth(self._pin, 0)

    def set_steering_dc(self, steering_dc):
        self._pi.set_servo_pulsewidth(self._pin, steering_dc)