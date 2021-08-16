#! /usr/bin/env python3
import pigpio
import os
import logging
import sys
import time
import numpy as np

logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)

class Motor:
    '''A brushless DC motor driven by one pin on a Raspberry Pi.'''

    def __init__ (self, pin, frequency=50):
        logging.debug("Motor.__init__")
        #  Configure GPIO
        if os.system ("pgrep pigpiod") != 0:
            logging.debug("Starting pipgiod")
            os.system("sudo pigpiod")

        self._pi = pigpio.pi()  # Connect to PIGPIO-Daemon
        self._pin = pin
        self._frequency = frequency
        self._speed = 0
        self._isArmed = False

        self._pi.set_mode(pin, pigpio.OUTPUT)
        self._pi.set_PWM_frequency(pin, frequency)
        self._pi.set_PWM_range(pin, 100)  # Control PWM from 0 .. 100 %
        self._pi.set_PWM_dutycycle(self._pin, 0)

    def speedperc2dc(self, mot_speed_perc):
        logging.debug("Motor.speedperc2dc")
        min_dc = 5      # Corresponds to 1 ms @ 50 Hz
        max_dc = 10     # Corresponds to 2 ms @ 50 Hz
        dc = min(max(7.5 + mot_speed_perc*2.5/100, min_dc), max_dc)
        print('DutyCycle: ' + str(dc))
        return dc

    def arm(self):
        logging.debug("Motor.arm")
        max_arming_throttle_dc = 8
        arming_throttle_dc_step = 1
        arming_throttle_dc_wait = 0.1

        self._pi.set_PWM_dutycycle(self._pin, 0)
        time.sleep(1)

        for i in np.arange(0,max_arming_throttle_dc,arming_throttle_dc_step):
            logging.debug("Motor DC: " + str(i))
            self._pi.set_PWM_dutycycle(self._pin, i)
            time.sleep(arming_throttle_dc_wait)
        
        for i in np.arange(max_arming_throttle_dc,3,-arming_throttle_dc_step):
            logging.debug("Motor DC: " + str(i))
            self._pi.set_PWM_dutycycle(self._pin, i)
            time.sleep(arming_throttle_dc_wait)

        self._pi.set_PWM_dutycycle(self._pin, 7.5)
        time.sleep(1)

        self._isArmed = True

    def set_motor_speed(self, speed):
        logging.debug("Motor.set_motor_speed")
        if not self._isArmed:
            return    
        self._pi.set_PWM_dutycycle(self._pin, self.speedperc2dc(speed))
        self._speed = speed
