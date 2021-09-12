#!/usr/bin/python3
import time
from pca9685 import PCA9685

PAN = 0
TILT = 1

pca9685 = PCA9685()
try:
    print ("This is an PCA9685 routine")    
    pca9685.setPWMFreq(50)
    pca9685.setRotationAngle(PAN, 90)
    pca9685.setRotationAngle(TILT, 90)
    
    while True:
        angle_tilt = int(input("Tilt angle [°]: "))
        angle_pan = int(input("Pan angle [°]: "))

        pca9685.setRotationAngle(TILT, angle_tilt)  
        pca9685.setRotationAngle(PAN, angle_pan)
        # for i in range(10,170,1): 
        #     pca9685.setRotationAngle(1, i)
        #     if(i<80):
        #         pca9685.setRotationAngle(0, i)   
        #     time.sleep(0.1)

        # for i in range(170,10,-1): 
        #     pca9685.setRotationAngle(1, i)   
        #     if(i<80):
        #         pca9685.setRotationAngle(0, i)            
        #     time.sleep(0.1)

except:
    pca9685.exit_PCA9685()
    print("\nProgram end")
    exit()