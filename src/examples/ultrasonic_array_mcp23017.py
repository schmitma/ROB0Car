#!/usr/bin/env python3

import time
import pigpio # https://abyz.me.uk/rpi/pigpio/python.html
import sys
import logging
import re

logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)

# Resources:
# - MCP23017 datasheet: https://ww1.microchip.com/downloads/en/devicedoc/20001952c.pdf

# TODO:
# - Use Banking mode -> implement Dictionary to switch between register address mapping
# - Use interrupts
# - Improve performance using parallelism -> check if parallelism improves speed since steps have to be carried out sequentially?
# - Apply coding guidelines, e.g. PEP8

# General process for interrupt driven operation
# 1. Trigger all sensors by setting all bits of GPIOA register to high
# 2. If interrupt occurs determine sensors for which echo signal has changed
#    by comparing old and new state of GPIOB register
# 3. If rising edge has been detected save start time
# 4. If falling edge has been detected save stop time and calculate distance

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

class HCSR04:
    def __init__(self, pos, gpio):
        # Maybe define position in coordinates?
        # x, y, z from center + angular orientation?
        self.position = pos
        self.gpio_assignment = gpio
        self.distance_cm = 0

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

    def __init__(self,
                 i2c_addr = MCP23017_I2C_ADDR, 
                 i2c_channel = 1, 
                 # Interrupt based operation is enabled if a GPIO to process
                 # the interrupt signal is supplied.
                 INTB_GPIO = None,
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
        self.pi=pigpio.pi() # Connect to local Pi.

        if not self.pi.connected:
            exit(0)

        self._cb = None
        self._INTB_GPIO = INTB_GPIO
        self._BANKING_MODE_IS_ACTIVE = BANKING_MODE_IS_ACTIVE
        
        self.sensors = [HCSR04("FRONT_RIGHT", 0),
                        HCSR04("FRONT_MIDDLE", 1)]#,
                        #HCSR04("FRONT_LEFT", 2),
                        #HCSR04("LEFT", 3),
                        #HCSR04("REAR_LEFT", 4),
                        #HCSR04("REAR_MIDDLE", 5),
                        #HCSR04("REAR_RIGHT", 6),
                        #HCSR04("RIGHT", 7)]
        
        self.number_of_sensors = len(self.sensors)
        self.sensor_bitmask = sum([1 << x.gpio_assignment for x in self.sensors])

        self._timeout = (2.0 * max_range_cm / 100.0) / HCSR04Cluster.SPEED_OF_SOUND

        if self._timeout < 0.001:
            self._timeout = 0.001
        elif self._timeout > 0.05:
            self._timeout = 0.05

        self._h = self.pi.i2c_open(i2c_channel, i2c_addr)
        self._set_banking_mode()

        # Initialise MCP23017 port A as outputs (trigger signal), B as inputs 
        # (echo signal).  According to MCP23017 datasheet a GPIO is defined as
        # input if the corresponding bit is set to 1
        self.pi.i2c_write_byte_data(self._h, 
            MCP23017_REGISTER_MAPPING["IODIRA"][self._BANKING_MODE_IS_ACTIVE], 
            0x00) # A is ouputs.
        self.pi.i2c_write_byte_data(self._h, 
            MCP23017_REGISTER_MAPPING["GPIOA"][self._BANKING_MODE_IS_ACTIVE], 
            0x00)
        self.pi.i2c_write_byte_data(self._h, 
            MCP23017_REGISTER_MAPPING["IODIRA"][self._BANKING_MODE_IS_ACTIVE], 
            0xFF) # B is inputs.

        self._bus_byte_micros = 1000000.0 / (i2c_kbps * 1000.0) * 9.0

        if INTB_GPIO is not None:
            # The GPINTEN register controls the interrupt-on-change feature
            # for each pin.  If a bit is set, the corresponding pin is enabled
            # for interrupt-on-change. Therefore, the bits of the GPINTENB
            # register for the installed sensors are set to 1.
            self.pi.i2c_write_byte_data(self._h,
                MCP23017_REGISTER_MAPPING["GPINTENB"][self._BANKING_MODE_IS_ACTIVE],
                self.sensor_bitmask)
            # The INTCON register controls how the associated pin value is
            # compared for the interrupt-on-change feature.  If a bit is set,
            # the corresponding I/O pin is compared against the associated bit
            # in the DEFVAL register.  If a bit value is clear, the 
            # corresponding I/O pin is compared against the previous value.
            # Since in this application an interrupt is to be triggered when
            # the echo signal of the corresponding sensor goes high or low,
            # the interrupt is configured to on-change.  Therefore the bits 
            # of the INTCONB register for the installed sensors are set to 0.
            self.pi.i2c_write_byte_data(self._h,
                MCP23017_REGISTER_MAPPING["INTCONB"][self._BANKING_MODE_IS_ACTIVE],
                0x00)
            # Since interrupt-on-change is configured, the DEFVALB register is set
            # to 0x00.
            self.pi.i2c_write_byte_data(self._h,
                MCP23017_REGISTER_MAPPING["DEFVALB"][self._BANKING_MODE_IS_ACTIVE],
                0x00)

            self.pi.set_mode(INTB_GPIO, pigpio.INPUT)

            self._cb = self.pi.callback(INTB_GPIO, pigpio.RISING_EDGE, self._cbf)
            self.GPIOB_state = self.pi.i2c_read_byte_data(self._h, 
                MCP23017_REGISTER_MAPPING["GPIOB"][self._BANKING_MODE_IS_ACTIVE])
            self._tick = [None] * self.number_of_sensors
            self._interrupt_processed = [False] * self.number_of_sensors

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
            MCP23017_REGISTER_MAPPING["IOCON1"][0], 
            iocon1_state)

    def _cbf(self, gpio, level, tick):
        """
        Each edge of the echo pin creates an interrupt and thus a rising
        edge on the interrupt pin.
        """
        logging.debug("HCSR04Cluster._cbf()")

        # How to determine which gpio of mcp23017 has changed?
        # Maybe save register state and compare on every interrupt?
        # Since a read acces to the register happens here the interrupt is 
        # already consumed!
        GPIOB_new_state = self.pi.i2c_read_byte_data(self._h, 
                MCP23017_REGISTER_MAPPING["GPIOB"][self._BANKING_MODE_IS_ACTIVE])
        
        state_diff = self.GPIOB_state ^ GPIOB_new_state
        
        # For all 1's in result which indicate a state change of the echo
        # signal process corresponding change
        # Treat state diff. bitmask as string and return indizes of 1's
        affected_sensors = [m.start() for m in re.finditer("1", str(bin(state_diff))[::-1])]

        for i in range(0, len(affected_sensors)):
            # Determine edge by comparing the binary string representation of
            # old and new state for corresponding sensor index
            if (str(bin(self.GPIOB_state))[::-1][affected_sensors[i]] == 0 and 
                str(bin(GPIOB_new_state))[::-1][affected_sensors[i]] == 1):
                # Rising edge of echo signal detected
                self._tick[affected_sensors[i]] = tick
            elif (str(bin(self.GPIOB_state))[::-1][affected_sensors[i]] == 1 and 
                str(bin(GPIOB_new_state))[::-1][affected_sensors[i]] == 0):
                # Falling edge of echo signal detected
                diff = pigpio.tickDiff(self._tick[affected_sensors[i]], tick)
                self.sensors[i].distance_cm = diff * HCSR04Cluster.MICS2CMS
                self._interrupt_processed[affected_sensors[i]] = True
            else:
                return -1

        self.GPIOB_state = GPIOB_new_state
            
    def trigger_measurement(self, sensor_number = None):
        logging.debug("HCSR04Cluster.trigger_measurement()")

        if self._INTB_GPIO is not None:
            self._interrupt_processed[:] = [False] * self.number_of_sensors

        if sensor_number is not None:
            bitmask = 1 << sensor_number
        else:
            bitmask = self.sensor_bitmask

        #  Send a 10us pulse
        logging.debug(f'Sending 10 us trigger pulse on GPIOA using bitmask {bin(bitmask)}')
        self.pi.i2c_write_byte_data(self._h, 
            MCP23017_REGISTER_MAPPING["GPIOA"][self._BANKING_MODE_IS_ACTIVE], 
            bitmask)
        time.sleep (10 * 10**-6)
        self.pi.i2c_write_byte_data(self._h, 
            MCP23017_REGISTER_MAPPING["GPIOA"][self._BANKING_MODE_IS_ACTIVE], 
            0x00)

    def measure_distance(self):
        """
        Triggers and returns an ultrasonic sensor measurement. The returned value is
        the distance in centimetres to the detected object.

        Sensor number is 0 for the sensor connected to A0/B0, 1 for the sensor
        connected to A1/B1 etc.
        """

        logging.debug("HCSR04Cluster.measure_distance()")

        if self._INTB_GPIO is not None:
            self.trigger_measurement()

            while not all(self._interrupt_processed):
                #if time.time() > timeout:
                #    return HCSR04Cluster.INVALID_READING
                pass

            return

        else:
            for i in range(self.number_of_sensors):
                self.trigger_measurement(i)
    
                GPIOB_state = self.pi.i2c_read_byte_data(self._h, 
                    MCP23017_REGISTER_MAPPING["GPIOB"][self._BANKING_MODE_IS_ACTIVE])
                logging.debug(f'Waiting for echo pin of sensor {i} to turn HIGH, GPIOB state: {bin(GPIOB_state)}')
                while (GPIOB_state & (1<<i)) == 0:
                    GPIOB_state = self.pi.i2c_read_byte_data(self._h, 
                        MCP23017_REGISTER_MAPPING["GPIOB"][self._BANKING_MODE_IS_ACTIVE])
                    logging.debug(f'Waiting for echo pin of sensor {i} to turn HIGH: {bin(GPIOB_state)}')
                    pass
                start = time.time()
            
                GPIOB_state = self.pi.i2c_read_byte_data(self._h, 
                    MCP23017_REGISTER_MAPPING["GPIOB"][self._BANKING_MODE_IS_ACTIVE])
                logging.debug(f'Waiting for echo pin of sensor {i} to turn HIGH: {bin(GPIOB_state)}')
                while (GPIOB_state & (1<<i)) == (1<<i):
                    GPIOB_state = self.pi.i2c_read_byte_data(self._h, 
                        MCP23017_REGISTER_MAPPING["GPIOB"][self._BANKING_MODE_IS_ACTIVE])
                    logging.debug(f'Waiting for echo pin of sensor {i} to turn HIGH: {bin(GPIOB_state)}')
                    pass
                end = time.time()

                self.sensors[i].distance_cm = ((end - start) * 34300) / 2

            return
    
    def print(self):
        for i in range(self.number_of_sensors):
            print(f'Measured distance of sensor {self.sensors[i].position}: {self.sensors[i].distance_cm} cm')

    def _is_is_power_of_two(self, n):
        # A utility function to check whether n is power of 2 or not.
        return (True if(n > 0 and ((n & (n - 1)) > 0))
                     else False)

    def _bitmask_to_sensor_idzs(self, bitmask):
        # Precondition: Only one bit is set
        # Determine sensor index by right shifting bitmask until value is
        # equal to 1.  The number of the necessary bit shifts is equal to 
        # the sensor index.
        if bin(bitmask).count("1") > 1 or self._is_is_power_of_two(bitmask) is not True:
            return -1
        else:
            idx = 0
            while bitmask is not 1:
                bitmask >>= 1
                idx +=1
            return idx
        
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

    s = ultrasonic_array_mcp23017.HCSR04Cluster(BANKING_MODE_IS_ACTIVE = 1,
                                                INTB_GPIO = 14)

    stop = time.time() + TIME

    while time.time() < stop:
        s.measure_distance()
        s.print()
        time.sleep(0.1)

    s.cancel()