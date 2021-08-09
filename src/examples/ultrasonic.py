#!/usr/bin/python3
import RPi.GPIO as GPIO
import time, sys

GPIO.setmode(GPIO.BOARD)
trig = [11, 36]
echo = [13, 38]

for i in range(len(trig)):
	GPIO.setup(echo[i], GPIO.IN)
	GPIO.setup(trig[i], GPIO.OUT)

def meas_dist(n):
	# print("Trigger Pin: ", trig[n], 
	    #   "\nEcho Pin: ", echo[n])
	GPIO.output(trig[n], True)
	time.sleep(0.00001) # 10 Mikrosekunden
	GPIO.output(trig[n], False)
	
	while GPIO.input(echo[n]) == 0:
		pass
	start = time.time()

	while GPIO.input(echo[n]) == 1:
		pass
	end = time.time()

	distance = ((end - start) * 34300) / 2
	print("Distance ", n, ": ", distance, " cm")
	return distance

while True:
	for i in range(len(trig)):
		meas_dist(i)
		time.sleep(0.5)
