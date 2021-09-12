#! /usr/bin/env python3
import pigpio
import os
import logging
import sys
import time
import numpy as np

logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)

# ESC Protocol settings
min_throttle_default_us = 1000
max_throttle_default_us = 2000
center_throttle_default_us = 1500

frequency_Hz = {
    'PWM': 200,          # max. frequency: 500 Hz
    'Oneshot125': 1000,  # max. frequency: 4 kHz
    'Oneshot42': 4000,   # max. frequency: 12 kHz
    'Multishot': 8000    # max. frequency: 32 kHz
}

divider = {
    'PWM': 1,
    'Oneshot125': 8,
    'Oneshot42': 24,
    'Multishot': 80
}

class Motor:
    '''A brushless DC motor driven by one pin on a Raspberry Pi.'''

    def __init__ (self, pin, protocol='PWM', isInverted=False):
        logging.debug("Motor.__init__")
        #  Configure GPIO
        if os.system ("pgrep pigpiod") != 0:
            logging.debug("Starting pipgiod")
            os.system("sudo pigpiod")

        self._pi = pigpio.pi()  # Connect to PIGPIO-Daemon
        self._pin = pin
        self._protocol = protocol

        # pigpio.set_PWM_frequency(): Returns the numerically closest frequency if OK, 
        # otherwise PI_BAD_USER_GPIO or PI_NOT_PERMITTED.
        self._frequency = self._pi.set_PWM_frequency(pin, frequency_Hz.get(self._protocol, 200))
        if self._frequency == pigpio.PI_BAD_USER_GPIO or self._frequency == pigpio.PI_NOT_PERMITTED:
            sys.exit('No valid frequency!')

        self._speed = 0
        self._max_speed_perc = 20
        self._isArmed = False
        self._isInverted = isInverted
        self._min_throttle_us = min_throttle_default_us / divider.get(self._protocol, 1)
        self._max_throttle_us = max_throttle_default_us / divider.get(self._protocol, 1)
        self._center_throttle_us = center_throttle_default_us / divider.get(self._protocol, 1)
        self._period_length_us = ((1/self._frequency) * (10 **6))
        self._min_dc = self._min_throttle_us / self._period_length_us * 100
        self._max_dc = self._max_throttle_us / self._period_length_us * 100
        self._center_dc = self._center_throttle_us / self._period_length_us * 100

        self._pi.set_mode(pin, pigpio.OUTPUT)
        self._pi.set_PWM_range(pin, 100)  # Control PWM from 0 .. 100 %
        self._pi.set_PWM_dutycycle(self._pin, 0)

    def speedperc2dc(self, mot_speed_perc):
        logging.debug("Motor.speedperc2dc")
        slope = (self._max_dc-self._center_dc)/100
        mot_speed_lim_perc = min(self._max_speed_perc, abs(mot_speed_perc)) * np.sign(mot_speed_perc)
        dc = self._center_dc + mot_speed_lim_perc * slope
        dc_lim = min(max(dc, self._min_dc), self._max_dc)
        print('DutyCycle: ' + str(dc))
        return dc_lim

    def arm(self):
        logging.debug("Motor.arm")
        max_arming_throttle_dc = 7.5
        arming_throttle_dc_step = 0.1
        arming_throttle_dc_wait = 0.1

        self._pi.set_PWM_dutycycle(self._pin, 0)
        time.sleep(1)

        self._pi.set_PWM_dutycycle(self._pin, self.speedperc2dc(80))
        time.sleep(1)
        self._pi.set_PWM_dutycycle(self._pin, self.speedperc2dc(30))
        time.sleep(1)

        self._isArmed = True

    def set_motor_speed(self, speed):
        logging.debug("Motor.set_motor_speed")
        if not self._isArmed:
            return
        if self._isInverted:
            speed *= -1    
        self._pi.set_PWM_dutycycle(self._pin, self.speedperc2dc(speed))
        self._speed = speed
