#!/usr/bin/python3
import os
import pigpio
import logging

logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)

class Steering:
    '''A servo motor driven by one pin on a Raspberry Pi.'''

    def __init__ (self, pin):
        logging.debug("Steering.__init__")
        #  Configure GPIO
        if os.system ("pgrep pigpiod") != 0:
            os.system("sudo pigpiod")

        self._pi = pigpio.pi()  # Connect to PIGPIO-Daemon
        self._pin = pin

        self._pi.set_mode(pin, pigpio.OUTPUT)
        self._pi.set_servo_pulsewidth(self._pin, 0)

    def set_steering_dc(self, steering_dc):
        logging.debug("Steering.set_steering_dc")
        self._pi.set_servo_pulsewidth(self._pin, steering_dc)