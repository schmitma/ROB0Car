#!/usr/bin/env python3

# i2c_sonar.py
# 2016-03-22
# Public Domain

import time

import pigpio # http://abyz.co.uk/rpi/pigpio/python.html

# To get fast sequential reads we use banked mode.  If banked mode isn't
# used then only half the accuracy will be achieved.

IODIRA=0x01
IPOLA=0x02
GPINTENA=0x04
DEFVALA=0x06
INTCONA=0x08
IOCON1=0x0A
GPPUA=0x0C
INTFA=0x0E
INTCAPA=0x10
GPIOA=0x12
OLATA=0x14

IODIRB=0x01
IPOLB=0x03
GPINTENB=0x05
DEFVALB=0x07
INTCONB=0x09
IOCON2=0x0B
GPPUB=0x0D
INTFB=0x0F
INTCAPB=0x11
GPIOB=0x13
OLATB=0x15

BANK=128
SEQOP=32

MODE = BANK + SEQOP

MCP_ADDR=0x27 # All address bits strapped high.

BUS_BYTE_TIME=90 # 90 for 100 kbps I2C bus, 22.5 for 400 kbps bus

TIME=30.0

pi=pigpio.pi() # Connect to local Pi.

if not pi.connected:
   exit(0)

h = pi.i2c_open(1, MCP_ADDR)

# Check to see if already initialised.

m1 = pi.i2c_read_byte_data(h, IOCON1)
m2 = pi.i2c_read_byte_data(h, IOCON2)

if (m1 != MODE) or (m2 != MODE):
   # Initialise to BANK + SEQOP
   pi.i2c_write_byte_data(h, 10, MODE)

# Initialise A0 as input, rest as outputs.

# A0 is used for the echo.
# B0 is used for the trigger.

pi.i2c_write_byte_data(h, IODIRA, 0x01)  # A0 is input
pi.i2c_write_byte_data(h, IODIRB, 0x00)  # everything else are outputs.

stop = time.time() + TIME

while time.time() < stop:

   # Send trigger on B0 then do 200 sequential reads of A0.

   count, data = pi.i2c_zip(h, 
      [7, 3, GPIOB, 1, 0, 7, 1, GPIOA, 6, 200])

   if data[0] == 0: # Ignore data if trigger start missed.
      f = False
      for i in range(count):
         v = data[i]  & 1 # Mask off all but echo bit.
         if f:
            if not v:
               print(int(round(i*BUS_BYTE_TIME/58.8)))
               break
         else:
            if v:
               f = True

   time.sleep(0.2)

pi.i2c_close(h)

pi.stop()