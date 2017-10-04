# Autonomous Robot
# TCES 360 2017
# Contributors:
#	-John Reagan Stovall
#	-Andrew Gates 
# Purpose:
# This the source code for an autonomous robot run using Raspberry Pi, connected with 
# Bricktronics Robotics Shield and using lego servo motors. The project also featrures 
# a pi-camera attached to two axis nerf canon to track and shoot specifically identified 
# targets. The Robot also features dinamic speed and object avoidance, keeps track of 
# amunition for main cannon as well as two smaller turets, and will navigate towards a 
# reload zone maked with a black and white cone. Targets are striped white spheres.

from __future__ import print_function
from __future__ import division
from builtins import input
from BrickPi import *   #import BrickPi.py file to use BrickPi operations
from multiprocessing import Process
import RPi.GPIO as GPIO
import time
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2

#--------------------MPU--------------------
import sys, getopt
sys.path.append('.')
import RTIMU
import time
import socket

IMU_IP = "127.0.0.2"
IMU_PORT = 5005

MON_IP = "127.0.0.5"
MON_PORT = 5005

SETTINGS_FILE = "RTIMULib"

s = RTIMU.Settings(SETTINGS_FILE)
imu = RTIMU.RTIMU(s)

# timers
t_print = time.time()
t_damp = time.time()
t_fail = time.time()
t_fail_timer = 0.0
t_shutdown = 0

if (not imu.IMUInit()):
    hack = time.time()
    imu_sentence = "$IIXDR,IMU_FAILED_TO_INITIALIZE*7C"
    if (hack - t_print) > 1.0:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(imu_sentence, (IMU_IP, IMU_PORT))
        t_print = hack
        t_shutdown += 1
        if t_shutdown > 9:
            sys.exit(1)

imu.setSlerpPower(0.02)
imu.setGyroEnable(True)
imu.setAccelEnable(True)
imu.setCompassEnable(True)

poll_interval = imu.IMUGetPollInterval()
#------------------MPU--------------------
magFound = False
#------------------TURN FUNCTIONS--------------------
def turn_180(right = 1):
    BrickPi.MotorSpeed[PORT_A] = -100 * right  #Set the speed of MotorA (-255 to 255)
    BrickPi.MotorSpeed[PORT_D] = 100 * right  #Set the speed of MotorB (-255 to 255)
    ot = time.time()
    while(time.time() - ot < .87):    #running while loop for 3 seconds
        BrickPiUpdateValues()       # Ask BrickPi to update values for sensors/motors
        time.sleep(.01)

    # Stop moving
    BrickPi.MotorSpeed[PORT_A] = 0
    BrickPi.MotorSpeed[PORT_D] = 0
    BrickPiUpdateValues()

def turn_90(right = 1):
    BrickPi.MotorSpeed[PORT_A] = -100 * right  #Set the speed of MotorA (-255 to 255)
    BrickPi.MotorSpeed[PORT_D] = 100 * right  #Set the speed of MotorB (-255 to 255)
    ot = time.time()
    wait = .72/2 + 0.11  # Half of 180
    while(time.time() - ot < wait):    #running while loop for 3 seconds
        BrickPiUpdateValues()       # Ask BrickPi to update values for sensors/motors
        time.sleep(.01)

    # Stop moving
    BrickPi.MotorSpeed[PORT_A] = 0
    BrickPi.MotorSpeed[PORT_D] = 0
    BrickPiUpdateValues()
#------------------TURN FUNCTIONS--------------------

#------------------BRICKPI STUFF--------------------
BrickPiSetup()  # setup the serial port for communication

LS_port_number = PORT_1
port_number = PORT_4	        # Define the port number here.
US2_port_number = PORT_2	# Define the port number here.  
US3_port_number = PORT_3	# Define the port number here.  

BrickPi.SensorType[port_number] = TYPE_SENSOR_ULTRASONIC_CONT   #Set the type of sensor at PORT_1
BrickPi.SensorType[US2_port_number] = TYPE_SENSOR_ULTRASONIC_CONT   #Set the type of sensor at PORT_1
BrickPi.SensorType[US3_port_number] = TYPE_SENSOR_ULTRASONIC_CONT   #Set the type of sensor at PORT_1
BrickPi.SensorType[LS_port_number] = TYPE_SENSOR_LIGHT_ON   #Set the type of sensor 

BrickPi.MotorEnable[PORT_A] = 1 #Enable the Motor A
BrickPi.MotorEnable[PORT_D] = 1 #Enable the Motor B

BrickPiSetupSensors()   #Send the properties of sensors to BrickPi

#Communication timeout in ms (how long since last valid communication before floating the motors).
#0 disables the timeout so the motor would keep running even if it is not communicating with the RaspberryPi
BrickPi.Timeout=3000
#print("BrickPiSetTimeout Status :",BrickPiSetTimeout())

preDist = 0
dist = 0
distL = 0
distR = 0
speedL = 1
speedR = 1
#------------------BRICKPI STUFF--------------------

#------------------CANNON STUFF--------------------
cannonShots = 4
turretShots = 6

# Use GPIO numbers not pin numbers
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Pin Definitons:
LLSPin = 23 # BCM pin Left Limit Switch 
RLSPin = 18 # BCM pin Right Limit Switch
SFPin  = 24 # BCM pin Shot Fired
ULSPin = 7  # BCM pin Upper Limit Switch
DLSPin = 25 # BCM pin Lower Limit Switch 


X1Pin = 22 # BCM pin x-axis Motor 1AY
X2Pin = 10 # BCM pin x-axis Motor 2AY
Y1Pin = 12  # BCM pin y-axis Motor 3AY
Y2Pin = 21 # BCM pin y-axis Motor 4AY
FCPin = 9  # BCM pin fire cannon
FTPin = 11 # BCM pin fire turret
LZRPin = 20 # BCM Laser

# set up the GPIO channels - 5 input and 5 output
GPIO.setup(LLSPin, GPIO.IN, GPIO.PUD_DOWN) # sets internal pulldown
GPIO.setup(RLSPin, GPIO.IN, GPIO.PUD_DOWN) # sets internal pulldown
GPIO.setup(SFPin,  GPIO.IN, GPIO.PUD_DOWN) # sets internal pulldown
GPIO.setup(ULSPin, GPIO.IN, GPIO.PUD_DOWN) # sets internal pulldown
GPIO.setup(DLSPin, GPIO.IN, GPIO.PUD_DOWN) # sets internal pulldown

GPIO.setup(X1Pin, GPIO.OUT) # Sets motor controller inputs
GPIO.setup(X2Pin, GPIO.OUT) # Sets motor controller inputs
GPIO.setup(Y1Pin, GPIO.OUT) # Sets motor controller inputs
GPIO.setup(Y2Pin, GPIO.OUT) # Sets motor controller inputs
GPIO.setup(FCPin,  GPIO.OUT) # Sets motor controller inputs
GPIO.setup(FTPin,  GPIO.OUT) # Sets motor controller inputs
GPIO.setup(LZRPin, GPIO.OUT) # Sets Lazer on or off 

# Initial states for Motors:
GPIO.output(X1Pin, GPIO.LOW)
GPIO.output(X2Pin, GPIO.LOW)
GPIO.output(Y1Pin, GPIO.LOW)
GPIO.output(Y2Pin, GPIO.LOW)
GPIO.output(FCPin, GPIO.LOW)
GPIO.output(FTPin, GPIO.LOW)
GPIO.output(LZRPin, GPIO.LOW)

# Functions
def lookLeft():
	print ("Looking Left")
	GPIO.output(X1Pin, GPIO.LOW)
	GPIO.output(X2Pin, GPIO.HIGH)
				
def lookRight():
        print ("Looking Right")
        GPIO.output(X1Pin, GPIO.HIGH)
        GPIO.output(X2Pin, GPIO.LOW)
      				
def lookUp():
	print ("Looking Up")
	GPIO.output(Y1Pin, GPIO.LOW)
	GPIO.output(Y2Pin, GPIO.HIGH)
	
		
def lookDown():
	print ("Looking Down")
	GPIO.output(Y1Pin, GPIO.HIGH)
	GPIO.output(Y2Pin, GPIO.LOW)
		
def stopX():
	print ("stop X")
	GPIO.output(X1Pin, GPIO.LOW)
	GPIO.output(X2Pin, GPIO.LOW)

def stopY():
        print ("stop Y")
        GPIO.output(Y1Pin, GPIO.LOW)
        GPIO.output(Y2Pin, GPIO.LOW)		
def stopC():
        print ("stop C")
        GPIO.output(FCPin, GPIO.LOW)
               
def stopT():
        print ("stop T")
        GPIO.output(FTPin, GPIO.LOW)

def fireCannon():
        shotFired = GPIO.LOW
        while ( shotFired == GPIO.LOW):
            GPIO.output(FCPin, GPIO.HIGH)
            time.sleep(.3)
            shotFired = GPIO.input(SFPin)
        GPIO.output(FCPin, GPIO.LOW)
        time.sleep(.01)
	#cannonShots = cannonShots - 1
		
		
def fireTurret():
	GPIO.output(FTPin, GPIO.HIGH)
	time.sleep(.075)	
	GPIO.output(FTPin, GPIO.LOW)
	#turretShots = turretShots - 1
		
def cannonStop():
        GPIO.output(X1Pin, GPIO.LOW)
        GPIO.output(X2Pin, GPIO.LOW)
        GPIO.output(Y1Pin, GPIO.LOW)
	GPIO.output(Y2Pin, GPIO.LOW)
	GPIO.output(FCPin, GPIO.LOW)
	GPIO.output(FTPin, GPIO.LOW)
	laserOff()
	time.sleep(0.1)	

def laserOn():
	GPIO.output(LZRPin, GPIO.HIGH)

def laserOff():
	GPIO.output(LZRPin, GPIO.LOW)
#------------------CANNON STUFF--------------------

#------------------OPENCV STUFF--------------------
#------------------OPENCV STUFF--------------------

def opencv():
    # initialize the camera and grab a reference to the raw camera capture
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(640, 480))
 
    # allow the camera to warmup 
    time.sleep(0.1)

    face_cascade = cv2.CascadeClassifier('spherecascade40x40.xml')
    #eye_cascade = cv2.CascadeClassifier('/usr/share/opencv/haarcascades/haarcascade_eye.xml')

    try:
        while(1):
            print ("in opencv")
            #rawCapture = PiRGBArray(camera, size=(640, 480))
            for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
                # grab the raw NumPy array representing the image, then initialize the timestamp
                # and occupied/unoccupied text
                image = frame.array
                gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)

                faces = face_cascade.detectMultiScale(gray, 5, 5)
                #eyes = eye_cascade.detectMultiScale(gray, 1.1, 5)

                print ("Found "+str(len(faces))+" face(s)")
                #print ("Found "+str(len(eyes))+" eyes(s)")

                for (x,y,w,h) in faces:
                    cv2.rectangle(image,(x,y),(x+w,y+h),(255,255,0),2)
                #for (x,y,w,h) in eyes:
                #        cv2.rectangle(image,(x,y),(x+w,y+h),(255,0,0),2)
     
                # show the frame
                cv2.imshow("Frame", image)
                key = cv2.waitKey(1) & 0xFF
     
                # clear the stream in preparation for the next frame
                rawCapture.truncate(0)

    except KeyboardInterrupt: # If CTRL+C is pressed, exit cleanly:
        print("Breaking")
def cannon():
    
    try:
        print("Cannon On")
        time.sleep(200)
        
        laserOn()
        lookRight()
        time.sleep(1)
        stopX()
        lookLeft()
        time.sleep(1)
        stopX()
        lookUp()
        time.sleep(.5)
        stopY()
        lookDown()
        time.sleep(.25)
        stopY()
        fireCannon()
        time.sleep(1)
        fireTurret()
        cannonStop()
        
    except KeyboardInterrupt: # If CTRL+C is pressed, exit cleanly:
        cannonStop()
        GPIO.cleanup() # cleanup all GPIO
        
def printing():
    while(1):
        print("hello")
        time.sleep(.1)

def navigation():
    preDist = 0
    dist = 0
    distL = 0
    distR = 0
    preDistL = 0
    preDistR = 0
    speedL = 1
    speedR = 1
    while(1):
        BrickPiUpdateValues()
        distL = BrickPi.Sensor[US2_port_number]
        distR = BrickPi.Sensor[US3_port_number]
        print("Front - ", dist, ", Left - ", distL, ", Right - ", distR)
                
        #BEGIN -------------------- Driving Correction
        if(abs(distL - distR) < 10):
            BrickPi.MotorSpeed[PORT_A] = -52  #Set the speed of MotorA (-255 to 255)
            BrickPi.MotorSpeed[PORT_D] = -50
            BrickPiUpdateValues()
        elif(distL < distR):
            if(speedR <= 1):
                speedR = 1
            else:
                speedR = speedR - 1

            #if(abs(preDistL - distL) == 0):
            #    speedL = speedL
            #else:
            speedL = speedL + 1
                
            if(speedL > 100):
                speedL = 100
            
            BrickPi.MotorSpeed[PORT_A] = -52
            BrickPi.MotorSpeed[PORT_D] = -50 - speedL
        else:
            if(speedL <= 1):
                speedL = 1
            else:
                speedL = speedL - 1

            #if(abs(preDistR - distR) == 0):
            #    speedR = speedR
            #else:
            speedR = speedR + 1
            
            if(speedR > 100):
                speedR = 100
            
            BrickPi.MotorSpeed[PORT_A] = -52 - speedR
            BrickPi.MotorSpeed[PORT_D] = -50

        print("speedL = ", speedL, ", speedR = ", speedR)
        preDistL = distL
        preDistR = distR
        #END -------------------- Driving Correction

        #BEGIN -------------------- Checking Magnetometer
        if imu.IMURead():
            data = imu.getIMUData()
            magnet = data["compass"]
            
            if(abs(magnet[0]) > 400 or abs(magnet[1]) > 400 or abs(magnet[2]) > 400):
                BrickPi.MotorSpeed[PORT_A] = 0
                BrickPi.MotorSpeed[PORT_D] = 0
                magFound = True
                tempMagnetReading = magnet[0]
                print("MAGNET - STOPPING")
                while(abs(tempMagnetReading) > 400):
                    if(imu.IMURead()):
                        data = imu.getIMUData()
                        magnet = data["compass"]
                        print(magnet)
                        print("waiting1")
                    tempMagnetReading = magnet[0]
                    time.sleep(poll_interval*1.0/1000.0)
                #BrickPi.MotorSpeed[PORT_C] = 0
        #END -------------------- Checking Magnetometer

        #BEGIN - Checking Light Sensor
        if(BrickPi.Sensor[LS_port_number] < 550):
            turn_90(-1)
            print("WHITE LINE - STOPPING")
        #END -------------------- Checking Light Sensor

        #BEGIN -------------------- Navigation
        else:
            #BEGIN -------------------- Checking Magnetometer
            if imu.IMURead():
                data = imu.getIMUData()
                magnet = data["compass"]
                print("mag ")
                print(magnet)
                
                if(abs(magnet[0]) > 500 or abs(magnet[1]) > 500 or abs(magnet[2]) > 500):
                    BrickPi.MotorSpeed[PORT_A] = 0
                    BrickPi.MotorSpeed[PORT_D] = 0
                    tempMagnetReading = magnet[0]
                    print("MAGNET - STOPPING")
                 
                    while(abs(tempMagnetReading) > 500):
                        if(imu.IMURead()):
                            data = imu.getIMUData()
                            magnet = data["compass"]
                            #print(magnet[0])
                            #print("waiting2")
                        tempMagnetReading = magnet[0]
                        time.sleep(poll_interval*1.0/1000.0)
                    BrickPi.MotorSpeed[PORT_C] = 0
            #END -------------------- Checking Magnetometer
            
            result = BrickPiUpdateValues()  # Ask BrickPi to update values for sensors/motors 
            print(result)
            if not result :
                dist = BrickPi.Sensor[port_number]
                #print ("dist")
                #print (dist)
                #print (preDist)

            # BEGIN -------------------- Turning
            if (dist < 25 and dist > 0 and preDist < 25 and preDist > 0):
                BrickPi.MotorSpeed[PORT_A] = 0  #Set the speed of MotorA (-255 to 255)
                BrickPi.MotorSpeed[PORT_D] = 0
                BrickPiUpdateValues()
                time.sleep(1)

                turn_90(1)
                time.sleep(1)
                
                result = BrickPiUpdateValues()  # Ask BrickPi to update values for sensors/motors 
                if not result :
                    distRight = BrickPi.Sensor[port_number]
                print (distRight)
                time.sleep(1)
                
                turn_180(-1)
                time.sleep(1)
                
                result = BrickPiUpdateValues()  # Ask BrickPi to update values for sensors/motors 
                if not result :
                    distLeft = BrickPi.Sensor[port_number]
                print (distLeft)
                time.sleep(1)
                
                if distLeft <= distRight:
                    turn_180()
                time.sleep(1)

                speedL = 1
                speedR = 1
            # END -------------------- Turning
            preDist = dist
        #END -------------------- Navigation

if __name__ == '__main__':
    #p = Process(target=printing)
    q = Process(target=navigation)
    r = Process(target=cannon)
    #s = Process(target=opencv)
    #p.start()
    q.start()
    r.start()
    #s.start()
    #p.join()
    q.join()
    r.join()
    #s.join()

