#!/usr/bin/python3
import time
import os
import pigpio
import sys
from pynput import keyboard

os.system ("sudo pigpiod")
time.sleep(1)

esc_speed_perc = 0

esc_frequency_hz = 50

steering_servo_bcmpin = 3
esc_left_bcmpin = 4
esc_right_bcmpin = 5

pi = pigpio.pi()  # Verbindung zum pigpio-Dämon

pi.set_mode(steering_servo_bcmpin, pigpio.OUTPUT)
pi.set_servo_pulsewidth(steering_servo_bcmpin, 0)

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

def on_press(key):
    global esc_speed_perc
    try:
        if key == keyboard.Key.left:
            pi.set_servo_pulsewidth(steering_servo_bcmpin, 1000)
        if key == keyboard.Key.right:
            pi.set_servo_pulsewidth(steering_servo_bcmpin, 2000)
        if key == keyboard.Key.up:
            fwd_speed_perc = 30
            esc_speed_perc = min(100, esc_speed_perc + 1)
            pi.set_PWM_dutycycle(esc_right_bcmpin, speedperc2dc(esc_speed_perc))
            pi.set_PWM_dutycycle(esc_left_bcmpin, speedperc2dc(esc_speed_perc))
            print(esc_speed_perc)
        if key == keyboard.Key.down:
            bwd_speed_perc = -30
            esc_speed_perc = max(-100, esc_speed_perc - 1)
            pi.set_PWM_dutycycle(esc_right_bcmpin, speedperc2dc(esc_speed_perc))
            pi.set_PWM_dutycycle(esc_left_bcmpin, speedperc2dc(esc_speed_perc))
            print(esc_speed_perc)

    except AttributeError:
        print('AttributeError')

def on_release(key):
    if key == keyboard.Key.left:
        pi.set_servo_pulsewidth(steering_servo_bcmpin, 1500)
    if key == keyboard.Key.right:
        pi.set_servo_pulsewidth(steering_servo_bcmpin, 1500)
    #if key == keyboard.Key.up:
        # pi.set_PWM_dutycycle(esc_right_bcmpin, speedperc2dc(0))
        # pi.set_PWM_dutycycle(esc_left_bcmpin, speedperc2dc(0))
    #if key == keyboard.Key.down:
        # pi.set_PWM_dutycycle(esc_right_bcmpin, speedperc2dc(0))
        # pi.set_PWM_dutycycle(esc_left_bcmpin, speedperc2dc(0))
    if key == keyboard.Key.esc:
        return False

listener = keyboard.Listener(on_press=on_press, on_release=on_release)

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

listener.start()

while True:
    True