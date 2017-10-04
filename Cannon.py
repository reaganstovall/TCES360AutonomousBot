#!/usr/bin/python

import RPi.GPIO as GPIO
import time 

#GPIO.setmode(GPIO.BCM)
GPIO.setmode(GPIO.BOARD)

#OutPut
enable = 7
motorX1 = 11
motorX2 = 15
motorY1 = 19
motorY2 = 21
motorFire = 23

#Input
leftLimitSwitch = 12
rightLimitSwitch = 16
shotFired = 18
upperLimitSwitch = 22
lowerLimitSwitch = 26

#Setup
GPIO.setup(enable, GPIO.OUT)
GPIO.setup(motorX1, GPIO.OUT)
GPIO.setup(motorX2, GPIO.OUT)
GPIO.setup(motorY1, GPIO.OUT)
GPIO.setup(motorY2, GPIO.OUT)
GPIO.setup(motorFire, GPIO.OUT)

GPIO.setup(leftLimitSwitch, GPIO.IN)
GPIO.setup(rightLimitSwitch, GPIO.IN)
GPIO.setup(shotFired, GPIO.IN)
GPIO.setup(upperLimitSwitch, GPIO.IN)
GPIO.setup(lowerLimitSwitch, GPIO.IN)

GPIO.output(enable, GPIO.HIGH)

# Functions 

def fireShot():
	while shotFired == FALSE :
		time.sleep(.01)## set to whatever is needed to work 
		GPIO.output(motorFire, GPIO.HIGH)
		
    return;




	
