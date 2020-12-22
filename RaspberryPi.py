#!/usr/bin/env python2.7
import RPi.GPIO as GPIO
import time
import sys, serial, struct
port = '/dev/ttyACM0'
sp = serial.Serial(port, baudrate=115200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
             xonxoff=False, rtscts=False, stopbits=serial.STOPBITS_ONE, timeout=None, dsrdtr=True)
sp.setDTR(True) # dsrdtr is ignored on Windows.

OFFSE_DUTY = 0.5        #define pulse offset of servo
SERVO_MIN_DUTY = 2.5+OFFSE_DUTY     #define pulse duty cycle for minimum angle of servo
SERVO_MAX_DUTY = 12.5+OFFSE_DUTY    #define pulse duty cycle for maximum angle of servo
panServoPin = 12
tiltServoPin = 13

def map( value, fromLow, fromHigh, toLow, toHigh):  # map a value from one range to another range
    return (toHigh-toLow)*(value-fromLow) / (fromHigh-fromLow) + toLow

def setup():
    global p
    global t
    GPIO.setmode(GPIO.BOARD)         # use PHYSICAL GPIO Numbering
    GPIO.setup(panServoPin, GPIO.OUT)   # Set servoPin to OUTPUT mode
    GPIO.output(panServoPin, GPIO.LOW)  # Make servoPin output LOW level
    GPIO.setup(tiltServoPin, GPIO.OUT)   # Set servoPin to OUTPUT mode
    GPIO.output(tiltServoPin, GPIO.LOW)  # Make servoPin output LOW leve
    
    p = GPIO.PWM(panServoPin, 50)  # set Frequece to 50Hz
    p.start(0)                     # Set initial Duty Cycle to 0
    t = GPIO.PWM(tiltServoPin, 50) # set Frequece to 50Hz
    t.start(0)                     # Set initial Duty Cycle to 0
    
def pServoWrite(angle):      # make the servo rotate to specific angle, 0-180 
    if(angle<0):
        angle = 0
    elif(angle > 180):
        angle = 180
    p.ChangeDutyCycle(map(angle,0,180,SERVO_MIN_DUTY,SERVO_MAX_DUTY)) # map the angle to duty cycle and output it

def tServoWrite(angle):      # make the servo rotate to specific angle, 0-180 
    if(angle<0):
        angle = 0
    elif(angle > 180):
        angle = 180
    t.ChangeDutyCycle(map(angle,0,180,SERVO_MIN_DUTY,SERVO_MAX_DUTY)) # map the angle to duty cycle and output it

def loop():
    xServo = 90
    yServo = 90
    pServoWrite(xServo)
    tServoWrite(yServo)
    
    while(True):
        xPos = ord(sp.read(1)[0])
        xErr = 80 - xPos
        
        yPos = ord(sp.read(1)[0])
        yErr = 60 - yPos
        
        xServo = xServo + 0.10*xErr
        yServo = yServo - 0.10*yErr
        
        if (xErr >= 5 or xErr <= -5):
            pServoWrite(xServo) # Pulse to PanServo
        else:
            pServoWrite(pServoWrite)
            
        if (yErr >= 5 or yErr <= -5):
            tServoWrite(yServo) # Pulse to tiltServo
        else:
            tServoWrite(tServoWrite)
            
        print("xErr: ", xErr, "yErr: ", yErr)
        
def destroy():
    p.stop()
    sp.close()
    GPIO.cleanup()

if __name__ == '__main__':     # Program entrance
    print ('Program is starting...')
    setup()
    try:
        loop()
    except KeyboardInterrupt:  # Press ctrl-c to end the program.
        destroy()

      
 
