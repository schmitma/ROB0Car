#!/usr/bin/python3
import time
import os
import pigpio
import sys
from pynput import keyboard

os.system ("sudo pigpiod")
time.sleep(1)

esc_frequency_hz = 50

esc_left_bcmpin = 4
esc_right_bcmpin = 5

pi = pigpio.pi()  # Verbindung zum pigpio-Dämon

pi.set_mode(esc_left_bcmpin, pigpio.OUTPUT)
pi.set_PWM_frequency(esc_left_bcmpin, esc_frequency_hz)
pi.set_PWM_range(esc_left_bcmpin, 100)

pi.set_mode(esc_right_bcmpin, pigpio.OUTPUT)
pi.set_PWM_frequency(esc_right_bcmpin, esc_frequency_hz)
pi.set_PWM_range(esc_right_bcmpin, 100)

time.sleep(1)

def speedperc2dc(mot_speed_perc):
    min_dc = 5
    max_dc = 10
    dc = min(max(7.5 + mot_speed_perc*2.5/100, min_dc), max_dc)
    print('DutyCycle: ' + str(dc))
    return dc

def arm(): #This is the arming procedure of an ESC 
    print("Connect the battery and press Enter")
    inp = input()    
    if inp == '':
        pi.set_PWM_dutycycle(esc_left_bcmpin, 0)
        pi.set_PWM_dutycycle(esc_right_bcmpin, 0)
        time.sleep(1)
        for i in range(0, 50, 5):
            pi.set_PWM_dutycycle(esc_left_bcmpin, speedperc2dc(i))
            pi.set_PWM_dutycycle(esc_right_bcmpin, speedperc2dc(i))
            print(i)
            time.sleep(0.1)
        
        for i in range(50, 0, -5):    
            pi.set_PWM_dutycycle(esc_left_bcmpin, speedperc2dc(i))
            pi.set_PWM_dutycycle(esc_right_bcmpin, speedperc2dc(i))
            print(i)
            time.sleep(0.1)

arm()

# Manuelle Eingabe des Duty Cycle in Dauerschleife
while True:
    dc = float(input("DC eingeben von 0 bis 100 %: "))
        
    pi.set_PWM_dutycycle(esc_left_bcmpin, dc)
    pi.set_PWM_dutycycle(esc_right_bcmpin, dc)
    print(dc)