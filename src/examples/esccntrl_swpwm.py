#!/usr/bin/python3
import RPi.GPIO as GPIO
import time
# Pin-Nummern verwenden (nicht GPIO-Nummern!)
GPIO.setmode(GPIO.BOARD)
# Pin 26 (= GPIO7) zur Datenauisgabe verwenden
pinl = 26
pinr = 40

GPIO.setup(pinl, GPIO.OUT)
GPIO.setup(pinr, GPIO.OUT)

# PWM-Instanz erstellen an Pin 26 mit Frequenz 50 Hz
pwml = GPIO.PWM(pinl, 50)
pwmr = GPIO.PWM(pinr, 50)

# PWM starten mit Duty Cycle von 50 Prozent 
pwml.start(0)
pwmr.start(0)

# Manuelle Eingabe des Duty Cycle in Dauerschleife
while True:
	dc = input("DC eingeben von 0 bis 100: ")
	pwml.ChangeDutyCycle(float(dc))
	pwmr.ChangeDutyCycle(float(dc))

# alle vom Script benutzten GPIOs/Pins wieder freigeben
GPIO.cleanup()
