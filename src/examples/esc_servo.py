#!/usr/bin/python3
import time
import os
import pigpio
import sys

os.system ("sudo pigpiod")
time.sleep(1)

servo = 3
pi = pigpio.pi()  # Verbindung zum pigpio-Dämon

esc_min_val = 1000
esc_zero_pos = 1500
esc_max_val = 2000

pi.set_mode(servo, pigpio.OUTPUT)
pi.set_servo_pulsewidth(servo, 0)
time.sleep(1)

def arm(): #This is the arming procedure of an ESC 
    print("Connect the battery and press Enter")
    inp = input()    
    if inp == '':
        pi.set_servo_pulsewidth(servo, 0)
        time.sleep(1)
        for i in range(1000, 1500, 50):
            pi.set_servo_pulsewidth(servo, i)
            print(i)
            time.sleep(0.1)
        
        for i in range(1500, 1000, -50):    
            pi.set_servo_pulsewidth(servo, i)
            print(i)
            time.sleep(0.1)

arm()

# Manuelle Eingabe des Duty Cycle in Dauerschleife
while True:
    pwm = int(input("DC eingeben von 1000 bis 2000 us: "))
    if pwm < 500:
        pwm = 500
    if pwm > 2500:
        pwm = 2500
        
    pi.set_servo_pulsewidth(servo, pwm)
    print(pwm)

