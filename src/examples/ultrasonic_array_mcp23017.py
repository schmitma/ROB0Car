#!/usr/bin/env python3

import time
import pigpio # https://abyz.me.uk/rpi/pigpio/python.html
import logging

logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)

class sensor:

    """
    Class to read up to 8 HC-SR04 ultrasonic sensors attached
    to a MCP23017 I2C port expander.

    Each sensor needs two GPIOs, one for the echo pin and one for
    the trigger pin.

    The MCP23017 has a total of 16 GPIOs: 8 on bank A and 8 on bank B

    Bank A is configured as output and should be connected
    to the sensor echo pins. Bank B is configured as input
    and should be connected to the sensor trigger pins.

    The sensors are referred to as sensor 0 to sensor 7. Sensor x
    should be connected to pin Ax (trigger) and pin Bx (echo).

    Two modes of operation are supported - interrupt driven or not
    interrupt driven. The interrupt driven mode is preferred but
    requires the use of one spare Pi GPIO. This GPIO should be
    connected to pin 20 (INTA) of the MCP23017.

    The range is reported in cms.  An invalid reading is
    reported as 9999.
    """
    # To get fast sequential banked mode is used. If banked
    # mode isn't used then only half the accuracy will be achieved.

    MCP23017_I2C_ADDR = 0x27

    IODIRA=0x00
    IODIRB=0x01

    IOCONA=0x0A
    IOCONB=0x0B

    GPIOA=0x12
    GPIOB=0x13
    
    # Value of TRIGGER_GAP has to be changed if interrupt-driven
    # mode is used and sensor values are incorrect.

    TRIGGER_GAP=430 # Microseconds between trigger and echo going high.

    SPEED_OF_SOUND = 340.29 # metres per second

    MICS2CMS = SPEED_OF_SOUND / 20000.0 # Allowing for round trip

    INVALID_READING=9999.0

    def __init__(self, pi, addr=MCP23017_I2C_ADDR, i2c_bus=1, INTA_GPIO=None,
                                  i2c_kbps=100.0, max_range_cms=450):
        """
        Instantiate with the Pi and the I2C address of the
        MCP23017.

        Optionally the I2C bus may be specified (default: 1).

        Optionally the GPIO connected to the interrupt line
        of the MCP23017 may be specified (default: None).

        Optionally the I2C bus speed may be specified (default
        100 kbps). This parameter should be specified if a different
        then standard I2C bus speed is used.

        Optionally the sensors maximum range in centimetres may
        be specified (default: 450).
        """

        self.pi = pi
        self._cb = None
        self._INTA_GPIO = INTA_GPIO

        self._timeout = (2.0 * max_range_cms / 100.0) / sensor.SPEED_OF_SOUND

        if self._timeout < 0.001:
            self._timeout = 0.001
        elif self._timeout > 0.05:
            self._timeout = 0.05

        self._h = pi.i2c_open(i2c_bus, addr)

        # # Check to see if already initialised.
        # mode_bank_a = pi.i2c_read_byte_data(self._h, sensor.IOCONA)
        # mode_bank_b = pi.i2c_read_byte_data(self._h, sensor.IOCONB)

        # if (mode_bank_a != sensor.MODE) or (mode_bank_b != sensor.MODE):
        #     # Initialise to BANK + SEQOP
        #     pi.i2c_write_byte_data(self._h, sensor.IOCONA, sensor.MODE)
        #     pi.i2c_write_byte_data(self._h, sensor.IOCONB, sensor.MODE)

        # Initialise A as outputs, B as inputs.

        # A is used for the trigger.
        # B is used for the echo.
        # According to MCP23017 datasheet (see https://ww1.microchip.com/downloads/en/devicedoc/20001952c.pdf)
        # a GPIO is defined as input if the corresponding bit is set to 1
        pi.i2c_write_byte_data(self._h, sensor.IODIRA, 0x00) # A is ouputs.
        pi.i2c_write_byte_data(self._h, sensor.IODIRB, 0xFF) # B is inputs.

        self._bus_byte_micros = 1000000.0 / (i2c_kbps * 1000.0) * 9.0

        if INTA_GPIO is not None:
            pi.i2c_write_byte_data(self._h, sensor.GPINTENA, 0x00) # Disable.
            pi.i2c_write_byte_data(self._h, sensor.DEFVALA, 0x00)  # N/A.
            pi.i2c_write_byte_data(self._h, sensor.INTCONA, 0x00)  # On change.

            pi.set_mode(INTA_GPIO, pigpio.INPUT)

            self._cb = pi.callback(INTA_GPIO, pigpio.RISING_EDGE, self._cbf)

            self._tick = None
            self._edge = 3
            self._micros = 0
            self._reading = False

            self._trigger_gap = int(sensor.TRIGGER_GAP / self._bus_byte_micros)-1
        else:
            bytes = int(self._timeout * 1000000.0 / self._bus_byte_micros)
            self._bytes_lsb = bytes & 0xFF
            self._bytes_msb = (bytes>>8) & 0xFF

    def _cbf(self, gpio, level, tick):
        """
        Each edge of the echo pin creates an interrupt and thus a rising
        edge on the interrupt pin.
        """
        if self._edge == 1:
            self._tick = tick
        elif self._edge == 2:
            diff = pigpio.tickDiff(self._tick, tick)
            self._micros = diff
            self._reading = True
        self._edge += 1

    def read(self, ranger):
        """
        Triggers and returns an ultrasonic sensor measurement. The returned value is
        the distance in centimetres to the detected object.

        sensor is 0 for the sensor connected to A0/B0, 1 for the sensor
        connected to A1/B1 etc.
        """
        if self._INTA_GPIO is not None:
            self._edge = 1
            self._reading = False

            count, data = self.pi.i2c_zip(self._h, 
                [7, 2, sensor.GPINTENA, 1<<ranger, # Interrupt on sensor.
                 7, 1, sensor.GPIOA,               # Clear interrupts.
                 6, 1,
                 7, 3, sensor.GPIOB, 1<<ranger, 0, # Send trigger.
                 7, 1, sensor.GPIOA,               # Consume interrupt.
                 6, self._trigger_gap])

            timeout = time.time() + self._timeout
            while not self._reading:
                if time.time() > timeout:
                    return sensor.INVALID_READING
                time.sleep(0.01)

            return self._micros * sensor.MICS2CMS

        else:
            #  Send a 10us pulse
            logging.debug(f'Sending 10 us trigger pulse on sensor: {hex(1<<ranger)}')
            self.pi.i2c_write_byte_data(self._h, sensor.GPIOA, 1<<ranger)
            time.sleep (0.00001)
            self.pi.i2c_write_byte_data(self._h, sensor.GPIOA, 0x00)

            #  Wait for echo to go high, then low
            StartTime = time.time()
            state = self.pi.i2c_read_byte_data(self._h, sensor.GPIOB)
            while (state & 1<<ranger) == 0:
                logging.debug(f'Waiting for echo pin to turn HIGH: {hex(state)}')
                state = self.pi.i2c_read_byte_data(self._h, sensor.GPIOB)
                StartTime = time.time()

            StopTime = time.time()
            state = self.pi.i2c_read_byte_data(self._h, sensor.GPIOB)
            while (state & 1<<ranger) == 1<<ranger:
                logging.debug(f'Waiting for echo pin to turn LOW: {hex(state)}')
                StopTime = time.time()
                if StopTime - StartTime >= 0.04:
                    StopTime = StartTime
                    break

            elapsed_time = StopTime - StartTime

        return elapsed_time * self.SPEED_OF_SOUND / 2

    def cancel(self):
        """
        Cancels the ultrasonic sensors and releases resources.
        """
        self.pi.i2c_close(self._h)
        if self._cb is not None:
            self._cb.cancel()

if __name__ == "__main__":

    import time

    import pigpio
    import ultrasonic_array_mcp23017

    TIME=60.0

    pi=pigpio.pi() # Connect to local Pi.

    if not pi.connected:
        exit(0)

    s = ultrasonic_array_mcp23017.sensor(pi, addr=0x27, INTA_GPIO=None, max_range_cms=200)

    stop = time.time() + TIME

    while time.time() < stop:
        v = s.read(0) # 0 is ranger 0 (connected to A0/B0).
        r = round(v,1)
        print(r)
        time.sleep(0.1)

    s.cancel()

    pi.stop()