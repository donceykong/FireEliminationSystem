# Doncey Albin
# 2/5/2021

import time
import board
import busio
import adafruit_lidarlite
import RPi.GPIO as GPIO
import time
import sys, serial, struct
import smtplib
from email.mime.text import MIMEText
from email.mime.image import MIMEImage
from email.mime.multipart import MIMEMultipart
from simple_pid import PID
pidx = PID(0.25, 0, 0.025, setpoint= 0)
pidy = PID(0.35, 0, 0.025, setpoint= 0)

# Create library object using our Bus I2C port for LiDAR
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_lidarlite.LIDARLite(i2c, reset_pin=None, configuration = 0, address=98)

# Set up for reading Serial Port
port = '/dev/ttyACM0'
sp = serial.Serial(port, baudrate=115200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
             xonxoff=False, rtscts=False, stopbits=serial.STOPBITS_ONE, timeout=None, dsrdtr=True)
sp.setDTR(True) # dsrdtr is ignored on Windows.

OFFSE_DUTY = 0.5        #define pulse offset of servo
SERVO_MIN_DUTY = 2.5+OFFSE_DUTY     #define pulse duty cycle for minimum angle of servo
SERVO_MAX_DUTY = 12.5+OFFSE_DUTY    #define pulse duty cycle for maximum angle of servo
panServoPin = 18
tiltServoPin = 27
tiltServo2Pin = 24

def setupEmail():
    global port 
    port = '/dev/ttyACM2'
    global sender_email 
    sender_email = "donceyalbin@gmail.com"
    global receiver_email 
    receiver_email = "donceyalbin@gmail.com"
    global password 
    password = "Donthackme2MF33"
    
    # Create a multipart message and set headers
    global message
    message = MIMEMultipart("alternative")
    message["Subject"] = "Fire Detected"
    message["From"] = sender_email
    message["To"] = receiver_email
    
    # Create the plain-text and HTML version of your message
    text = """\
    FIRE HAS BEEN DETECTED.
    See below:"""
    html = """\
    <html>
      <body>
        <p>A FIRE HAS BEEN DETECTED<br>
           Please view image below:<br>
        </p>
      </body>
    </html>
    """
    
    # Turn these into plain/html MIMEText objects
    part1 = MIMEText(text, "plain")
    part2 = MIMEText(html, "html")
    
    # Add HTML/plain-text parts to MIMEMultipart message
    # The email client will try to render the last part first
    message.attach(part1)
    message.attach(part2)

def map( value, fromLow, fromHigh, toLow, toHigh):  # map a value from one range to another range
    return (toHigh-toLow)*(value-fromLow) / (fromHigh-fromLow) + toLow

def bootloader_reset():
    __BOOTLDR_RESET = 0xABCD0002
    sp.write(struct.pack("<I", __BOOTLDR_RESET))
    
def setup():
    global p
    global t
    global t2
    GPIO.setmode(GPIO.BCM)            # use PHYSICAL GPIO Numbering
    GPIO.setwarnings(False)             # Remove "already in use warning"
    GPIO.setup(panServoPin, GPIO.OUT)   # Set panServoPin to OUTPUT mode
    GPIO.output(panServoPin, GPIO.LOW)  # Make panServoPin output LOW level
    GPIO.setup(tiltServoPin, GPIO.OUT)  # Set tiltServoPin to OUTPUT mode
    GPIO.output(tiltServoPin, GPIO.LOW) # Make tiltServoPin output LOW level
    GPIO.setup(tiltServo2Pin, GPIO.OUT)  # Set tiltServo2Pin to OUTPUT mode
    GPIO.output(tiltServo2Pin, GPIO.LOW) # Make tiltServo2Pin output LOW level
    
    p = GPIO.PWM(panServoPin, 50)  # set Frequece to 50Hz
    p.start(0)                     # Set initial Duty Cycle to 0
    t = GPIO.PWM(tiltServoPin, 50) # set Frequece to 50Hz
    t.start(0)                     # Set initial Duty Cycle to 0
    t2 = GPIO.PWM(tiltServo2Pin, 50) # set Frequece to 50Hz
    t2.start(0)                     # Set initial Duty Cycle to 0
    
def pServoWrite(angle):      # make the pan servo rotate to specific angle, 0-180 
    if(angle<0):
        angle = 0
    elif(angle > 180):
        angle = 180
    p.ChangeDutyCycle(map(angle,0,180,SERVO_MIN_DUTY,SERVO_MAX_DUTY)) # map the angle to duty cycle and output it

def tServoWrite(angle):      # make the tilt servo rotate to specific angle, 0-180 
    if(angle<0):
        angle = 0
    elif(angle > 180):
        angle = 180
    t.ChangeDutyCycle(map(angle,0,180,SERVO_MIN_DUTY,SERVO_MAX_DUTY)) # map the angle to duty cycle and output it
    
def t2ServoWrite(angle):      # make the tilt servo rotate to specific angle, 0-180 
    if(angle<0):
        angle = 0
    elif(angle > 180):
        angle = 180
    t2.ChangeDutyCycle(map(angle,0,180,SERVO_MIN_DUTY,SERVO_MAX_DUTY)) # map the angle to duty cycle and output it

def loop():
    fireDet = False
    xServo = 0
    yServo = 90
    y2Servo = 90
   
    pServoWrite(xServo)
    tServoWrite(yServo)
    t2ServoWrite(y2Servo)
    
    time.sleep(1)
    panCamera(fireDet)
    servoControl()

def panCamera(fireDet):
    while(not fireDet):
        try:
            # We print tuples so you can plot with Mu Plotter
            print((sensor.distance - 10)/2.54) # Distance in inches. Please note the -10 is for correcting the sensor measurement in cm.
        except RuntimeError as e:
            # If we get a reading error, just print it and keep truckin'
            print(e)

        print("pan left")
        for i in range(0, 180, 3):
            data = sp.read(1)[0]
            xServo = i
            pServoWrite(xServo)
            if(not(data == 244)):
                print("fire Detected")
                fireDet = True
                servoControl(xServo, 90, 90, 0)
                break
            #time.sleep(0.01)
            
        if (fireDet):
            continue
            
        print("pan right")
        for i in range(180, 0, -3):
            data = sp.read(1)[0]
            xServo = i
            pServoWrite(xServo) 
            if(not(data == 244)):
                print("fire Detected")
                fireDet = True
                servoControl(xServo, 90, 90, 0)
                break
            #time.sleep(0.01)
    
def servoControl(xServo, yServo, y2Servo, emailNum):
    sp.close()
    sp.open()
    while(True):
        xPos = sp.read(1)[0]
        xErr = 20 - xPos
        
        yPos = sp.read(1)[0]
        yErr = 15 - yPos
        
        xPGain = 0.06
        yPGain = 0.06
        
        xServo = xServo - pidx(xErr) #xServo + (xPGain*xErr)
        yServo = yServo + pidy(yErr) #yServo - (yPGain*yErr)
        y2Servo = y2Servo - pidy(yErr) #yServo + (yPGain*yErr)
        
        pServoWrite(xServo)
        tServoWrite(yServo)
        t2ServoWrite(y2Servo)
        
#        print("yServo: ", yServo, "y2Servo: ", y2Servo)
#        print("xErr: ", xErr, "yErr: ", yErr)
        
        if((xErr <= 1 and xErr >= -1) and (yErr <= 1 and yErr >= -1) and emailNum == 0):
                emailPhoto(xServo, yServo, y2Servo)
        
def destroy():
    p.stop()
    t.stop()
    sp.close()
    GPIO.cleanup()

def emailPhoto(xServo, yServo, y2Servo):
     sp.flush()
     i = 0
     while (True):
         #sp.write("snap")
         print("emailing photo")
         size = struct.unpack('<L', sp.read(4))[0]
         img = sp.read(size)
         print("photo unpacked")
         
         if (True):
              i = i + 1
              print("fire detected " + str(i) + " times.")
              with open("img" + str(i) + ".jpg", "wb") as f:
                   f.write(img)
                   
              # We assume that the image file is in the same directory that you run your Python script from
              fp = open("img" + str(i) + ".jpg", 'rb')
              image = MIMEImage(fp.read(),_subtype="jpg")
              fp.close()
              
              # Specify the  ID according to the img src in the HTML part
              image.add_header('Content-ID', '<Mailtrapimage>')
              message.attach(image)
              
              # Log in to server using secure context and send email
              server = smtplib.SMTP_SSL("smtp.gmail.com", 465)
              server.login(sender_email, password)
              server.sendmail(sender_email, receiver_email, message.as_string())
              #server.quit()
              
              sp.flush()
              servoControl(xServo, yServo, y2Servo, 1)

          
if __name__ == '__main__':     # Program entrance
    print ('Fire detection system is starting...')
    setup()
    setupEmail()
    try:
        loop()
    except KeyboardInterrupt:  # Press ctrl-c to end the program.
        destroy()

