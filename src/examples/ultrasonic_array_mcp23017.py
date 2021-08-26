#!/usr/bin/env python3

import time
import pigpio # https://abyz.me.uk/rpi/pigpio/python.html
import sys
import logging

logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)

# TODO:
# - Use Banking mode -> implement Dictionary to switch between register address mapping
# - Use interrupts
# - Improve performance using parallelism
# - Apply coding guidelines, e.g. PEP8

MCP23017_I2C_ADDR = 0x27

# According to the MCP23017 datasheet:
# The 16-bit I/O port functionally consists of two 8-bit ports (PORTA and 
# PORTB).  The MCP23X17 can be configured to operate in the 8-bit or 16-bit 
# modes via IOCON.BANK.
#
# The BANK bit changes how the registers are mapped:
# If BANK = 1, the registers associated with each port are segregated.
# Registers associated with PORTA are mapped from address 00h - 0Ah and
# registers associated with PORTB are mapped from 10h - 1Ah.
# If BANK = 0, the A/B registers are paired.  For example, IODIRA is mapped
# to address 00h and IODIRB is mapped to the next address (address 01h). 
# The mapping for all registers is from 00h -15h.
#
# To take this into account the register mapping is implemented as a
# dictionary, using the register name as key. The corresponding value is a
# list containing the register address for 8-bit mode at index 0 and 
# containing the register address for 16-bit mode (banked-mode) at
# index 1.
MCP23017_REGISTER_MAPPING = {
    "IODIRA":   [0x00, 0x00], 
    "IODIRB":   [0x01, 0x10],
    "IPOLA":    [0x02, 0x01],
    "IPOLB":    [0x03, 0x11],
    "GPINTENA": [0x04, 0x02],
    "GPINTENB": [0x05, 0x12],
    "DEFVALA":  [0x06, 0x03],
    "DEFVALB":  [0x07, 0x13],
    "INTCONA":  [0x08, 0x04],
    "INTCONB":  [0x09, 0x14],
    "IOCON1":   [0x0A, 0x05],
    "IOCON2":   [0x0B, 0x15],
    "GPPUA":    [0x0C, 0x06],
    "GPPUB":    [0x0D, 0x16],
    "INTFA":    [0x0E, 0x07],
    "INTFB":    [0x0F, 0x17],
    "INTCAPA":  [0x10, 0x08],
    "INTCAPB":  [0x11, 0x18],
    "GPIOA":    [0x12, 0x09],
    "GPIOB":    [0x13, 0x19],
    "OLATA":    [0x14, 0x0A],
    "OLATB":    [0x15, 0x1A]
}

# MCP23017 configuration register flags (IOCON ADDR 0x05)
BANK   = (1<<7)
SEQOP  = (1<<5)
INTPOL = (1<<1)

class HCSR04Cluster:

    """
    Class to read up to 8 HC-SR04 ultrasonic sensors attached
    to a MCP23017 I2C port expander.

    Each sensor needs two GPIOs, one for the echo pin and one for
    the trigger pin.

    The MCP23017 has a total of 16 GPIOs: 8 on bank A and 8 on bank B

    Bank A is configured as output and should be connected
    to the sensor echo pins.  Bank B is configured as input
    and should be connected to the sensor trigger pins.

    The sensors are referred to as sensor 0 to sensor 7. Sensor x
    should be connected to pin Ax (trigger) and pin Bx (echo).

    Two modes of operation are supported - interrupt driven or not
    interrupt driven.  The interrupt driven mode is preferred but
    requires the use of one spare Pi GPIO. This GPIO should be
    connected to pin 20 (INTA) of the MCP23017.

    The range is reported in cms.  An invalid reading is
    reported as 9999.
    """
    # To get fast sequential banked mode is used. If banked
    # mode isn't used then only half the accuracy will be achieved.

    MCP23017_I2C_ADDR = 0x27
    
    IOCONA=0x0A
    IOCONB=0x0B

    GPIOA=0x12
    GPIOB=0x13
    
    IODIRA=0x00
    IODIRB=0x01

    IOCONA=0x0A
    IOCONB=0x0B

    GPIOA=0x12
    GPIOB=0x13

    GPINTENA=0x02
    DEFVALA=0x03
    INTCONA=0x04
    
    # Value of TRIGGER_GAP has to be changed if interrupt-driven
    # mode is used and sensor values are incorrect.

    TRIGGER_GAP=430 # Microseconds between trigger and echo going high.

    SPEED_OF_SOUND = 340.29 # metres per second

    MICS2CMS = SPEED_OF_SOUND / 20000.0 # Allowing for round trip

    INVALID_READING=9999.0

    def __init__(self, pi, 
                 i2c_addr = MCP23017_I2C_ADDR, 
                 i2c_channel = 1, 
                 INTA_GPIO = None,
                 BANKING_MODE_IS_ACTIVE = 0,
                 i2c_kbps = 100.0, 
                 max_range_cm = 450):
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
        self._BANKING_MODE_IS_ACTIVE = BANKING_MODE_IS_ACTIVE

        self._timeout = (2.0 * max_range_cm / 100.0) / HCSR04Cluster.SPEED_OF_SOUND

        if self._timeout < 0.001:
            self._timeout = 0.001
        elif self._timeout > 0.05:
            self._timeout = 0.05

        self._h = pi.i2c_open(i2c_channel, i2c_addr)
        self._set_banking_mode()

        # Initialise A as outputs, B as inputs.
        # A is used for the trigger.
        # B is used for the echo.
        # According to MCP23017 datasheet (see https://ww1.microchip.com/downloads/en/devicedoc/20001952c.pdf)
        # a GPIO is defined as input if the corresponding bit is set to 1
        pi.i2c_write_byte_data(self._h, 
            MCP23017_REGISTER_MAPPING["IODIRA"][self._BANKING_MODE_IS_ACTIVE], 
            0x00) # A is ouputs.
        pi.i2c_write_byte_data(self._h, 
            MCP23017_REGISTER_MAPPING["GPIOA"][self._BANKING_MODE_IS_ACTIVE], 
            0x00)
        pi.i2c_write_byte_data(self._h, 
            MCP23017_REGISTER_MAPPING["IODIRA"][self._BANKING_MODE_IS_ACTIVE], 
            0xFF) # B is inputs.

        self._bus_byte_micros = 1000000.0 / (i2c_kbps * 1000.0) * 9.0

        if INTA_GPIO is not None:
            pi.i2c_write_byte_data(self._h, HCSR04Cluster.GPINTENA, 0x00) # Disable.
            pi.i2c_write_byte_data(self._h, HCSR04Cluster.DEFVALA, 0x00)  # N/A.
            pi.i2c_write_byte_data(self._h, HCSR04Cluster.INTCONA, 0x00)  # On change.

            pi.set_mode(INTA_GPIO, pigpio.INPUT)

            self._cb = pi.callback(INTA_GPIO, pigpio.RISING_EDGE, self._cbf)

            self._tick = None
            self._edge = 3
            self._micros = 0
            self._reading = False

            self._trigger_gap = int(HCSR04Cluster.TRIGGER_GAP / self._bus_byte_micros)-1
        else:
            bytes = int(self._timeout * 1000000.0 / self._bus_byte_micros)
            self._bytes_lsb = bytes & 0xFF
            self._bytes_msb = (bytes>>8) & 0xFF

    def _set_banking_mode(self):
        """
        Since the state of IOCON.BANK cannot be determined reliably, the
        device must first be brought into a defined state using the following
        procedure.  Afterwards the state can be set according to the value of 
        class attribute.

        See https://electronics.stackexchange.com/questions/325916/mcp23017-detecting-state-of-iocon-bank-bit-after-mcu-reset/325917
        for reference.
        """
        # Assume MCP23017 is in banking mode, i.e. IOCON.BANK = 1 and read 
        # state of IOCON configuration register (ADDR 0x05).
        iocon1_state = self.pi.i2c_read_byte_data(self._h, 
            MCP23017_REGISTER_MAPPING["IOCON1"][1])
        
        # Clear bit 7 of IOCON configuration register and write value back.
        # If MCP23017 has been in banking mode, device has now switched
        # from BANK = 1 to BANK = 0. If MCP23017 has been in 8-bit mode,
        # GPINTENB.GPINT7 has been disabled. Anyway the MCP23017 is now in 
        # the defined state IOCON.BANK = 0 an can be set according to the
        # value of the class attribute.
        iocon1_state &= ~BANK
        self.pi.i2c_write_byte_data(self._h, 
            MCP23017_REGISTER_MAPPING["IOCON1"][1], 
            iocon1_state)

        iocon1_state |= self._BANKING_MODE_IS_ACTIVE * BANK
        self.pi.i2c_write_byte_data(self._h, 
            MCP23017_REGISTER_MAPPING["IOCON1"][1], 
            iocon1_state)

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
                [7, 2, HCSR04Cluster.GPINTENA, 1<<ranger, # Interrupt on sensor.
                 7, 1, HCSR04Cluster.GPIOA,               # Clear interrupts.
                 6, 1,
                 7, 3, HCSR04Cluster.GPIOB, 1<<ranger, 0, # Send trigger.
                 7, 1, HCSR04Cluster.GPIOA,               # Consume interrupt.
                 6, self._trigger_gap])

            timeout = time.time() + self._timeout
            while not self._reading:
                if time.time() > timeout:
                    return HCSR04Cluster.INVALID_READING
                time.sleep(0.01)

            return self._micros * HCSR04Cluster.MICS2CMS

        else:
            logging.debug("meas_dist()")

            #  Send a 10us pulse
            logging.debug(f'Sending 10 us trigger pulse on GPIOA: {hex(1<<ranger)}')
            self.pi.i2c_write_byte_data(self._h, 
                MCP23017_REGISTER_MAPPING["GPIOA"][self._BANKING_MODE_IS_ACTIVE], 
                1<<ranger)
            time.sleep (0.00001)
            self.pi.i2c_write_byte_data(self._h, 
                MCP23017_REGISTER_MAPPING["GPIOA"][self._BANKING_MODE_IS_ACTIVE], 
                0x00)
            
            state = self.pi.i2c_read_byte_data(self._h, 
                MCP23017_REGISTER_MAPPING["GPIOB"][self._BANKING_MODE_IS_ACTIVE])
            logging.debug(f'Waiting for echo pin of sensor {ranger} to turn HIGH: {hex(state)}')
            while (state & (1<<ranger)) == 0:
                state = self.pi.i2c_read_byte_data(self._h, 
                    MCP23017_REGISTER_MAPPING["GPIOB"][self._BANKING_MODE_IS_ACTIVE])
                logging.debug(f'Waiting for echo pin of sensor {ranger} to turn HIGH: {hex(state)}')
                pass
            start = time.time()
        
            state = self.pi.i2c_read_byte_data(self._h, 
                MCP23017_REGISTER_MAPPING["GPIOB"][self._BANKING_MODE_IS_ACTIVE])
            logging.debug(f'Waiting for echo pin of sensor {ranger} to turn HIGH: {hex(state)}')
            while (state & (1<<ranger)) == (1<<ranger):
                state = self.pi.i2c_read_byte_data(self._h, 
                    MCP23017_REGISTER_MAPPING["GPIOB"][self._BANKING_MODE_IS_ACTIVE])
                logging.debug(f'Waiting for echo pin of sensor {ranger} to turn HIGH: {hex(state)}')
                pass
            end = time.time()
        
            distance = ((end - start) * 34300) / 2
            print("Distance of sensor ", ranger, ": ", distance, " cm")
            return distance
        
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

    s = ultrasonic_array_mcp23017.HCSR04Cluster(pi, 
        BANKING_MODE_IS_ACTIVE = 1)

    stop = time.time() + TIME

    while time.time() < stop:
        v = s.read(0) # 0 is ranger 0 (connected to A0/B0).
        time.sleep(0.1)

        v = s.read(1) # 1 is ranger 1 (connected to A1/B1).
        time.sleep(0.1)

    s.cancel()

    pi.stop()