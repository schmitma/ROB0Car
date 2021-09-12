#!/usr/bin/python3
import picamera, sys, time
from pynput import keyboard
import RPi.GPIO as GPIO
cam = picamera.PiCamera()
cam.resolution = (1920, 1080)

def captureImage():
    t = time.strftime("%Y_%m_%d-%H_%M_%S")
    print("Capture Image...")
    cam.capture('/tmp/IMAGE_%s.jpg' %t)
    print("Image saved.")

def on_press(key):
    try:
        if key == keyboard.Key.up:
            captureImage()
    except AttributeError:
        print('AttributeError')

listener = keyboard.Listener(on_press=on_press)

listener.start()

try:
    while True:
        time.sleep(5)
except KeyboardInterrupt:
    sys.exit()
    cam.close()