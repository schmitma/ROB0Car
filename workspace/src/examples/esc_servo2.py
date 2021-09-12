#!/usr/bin/python3
import time
import os
import pigpio
import sys
from pynput import keyboard

os.system ("sudo pigpiod")
time.sleep(1)


esc_left_bcmpin = 5
esc_right_bcmpin = 4

pi = pigpio.pi()  # Verbindung zum pigpio-Dämon

pi.set_mode(steering_servo_bcmpin, pigpio.OUTPUT)
pi.set_servo_pulsewidth(steering_servo_bcmpin, 0)
time.sleep(1)

pi.set_mode(esc, pigpio.OUTPUT)
pi.set_PWM_frequency(esc, 50)
time.sleep(1)

def arm(): #This is the arming procedure of an ESC 
    print("Connect the battery and press Enter")
    inp = input()    
    if inp == '':
        pi.set_servo_pulsewidth(esc_left_bcmpin, 0)
        time.sleep(1)
        for i in range(1000, 1500, 50):
            pi.set_servo_pulsewidth(esc_left_bcmpin, i)
            print(i)
            time.sleep(0.1)
        
        for i in range(1500, 1000, -50):    
            pi.set_servo_pulsewidth(esc_left_bcmpin, i)
            print(i)
            time.sleep(0.1)

#arm()

listener.start()

while True:
    True