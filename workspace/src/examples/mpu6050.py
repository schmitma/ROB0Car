#!/usr/bin/python3
import smbus
import math
import time

# Registers
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c

def read_bytes(reg):
    return bus.read_byte_data(address, reg)

def read_word(reg):
    h = bus.read_byte_data(address, reg)
    l = bus.read_byte_data(address, reg+1)
    value = (h<<8) + l
    return value
def read_word_2c(reg):
    val = read_word(reg)
    if(val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def dist(a, b):
    return math.sqrt((a*a)+(b*b))

def get_y_rotation(x, y, z):
    radians = math.atan2(x, dist(y, z))
    return -math.degrees(radians)

def get_x_rotation(x, y, z):
    radians = math.atan2(y, dist(x, z))
    return math.degrees(radians)

bus = smbus.SMBus(1)
address = 0x68 # via i2cdetect -y 1

# Activate module
bus.write_byte_data(address, power_mgmt_1, 0)

def print_values():
    print("Gyro")
    print("----------------")

    gyro_x = read_word_2c(0x43)
    gyro_y = read_word_2c(0x45)
    gyro_z = read_word_2c(0x47)

    print("Gyro X-Direction: ", ("%5d" % gyro_x), ", Scaled: ", (gyro_x / 131))
    print("Gyro Y-Direction: ", ("%5d" % gyro_y), ", Scaled: ", (gyro_y / 131))
    print("Gyro Z-Direction: ", ("%5d" % gyro_z), ", Scaled: ", (gyro_z / 131))

    print("")
    print("Acceleration")
    print("----------------")

    accel_x = read_word_2c(0x3b)
    accel_y = read_word_2c(0x3d)
    accel_z = read_word_2c(0x3f)

    accel_x_scaled = accel_x / 16384.0
    accel_y_scaled = accel_y / 16384.0
    accel_z_scaled = accel_z / 16384.0

    print("Acceleration in X-Direction: ", ("%6d" % accel_x), ", Scaled: ", accel_x_scaled)
    print("Acceleration in Y-Direction: ", ("%6d" % accel_y), ", Scaled: ", accel_y_scaled)
    print("Acceleration in Z-Direction: ", ("%6d" % accel_z), ", Scaled: ", accel_z_scaled)

    print("X-Rotation: ", get_x_rotation(accel_x_scaled, accel_y_scaled, accel_z_scaled))
    print("Y-Rotation: ", get_y_rotation(accel_x_scaled, accel_y_scaled, accel_z_scaled))

while True:
    print_values()
    time.sleep(1)
