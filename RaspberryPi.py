Doncey Albin
# 3/4/2021

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
pidx = PID(0.25, 0, 0, setpoint= 0)
pidy = PID(0.30, 0, 0, setpoint= 0)

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
tiltServoPin = 24
tiltServo2Pin = 27

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

def openmv_reset():
    sp.write(b'snap')
    sp.flush()

def servo_setup():
    global p
    global t
    global t2
    GPIO.setmode(GPIO.BCM)               # use PHYSICAL GPIO Numbering
    GPIO.setwarnings(False)              # Remove "already in use warning"
    GPIO.setup(panServoPin, GPIO.OUT)    # Set panServoPin to OUTPUT mode
    GPIO.output(panServoPin, GPIO.LOW)   # Make panServoPin output LOW level
    GPIO.setup(tiltServoPin, GPIO.OUT)   # Set tiltServoPin to OUTPUT mode
    GPIO.output(tiltServoPin, GPIO.LOW)  # Make tiltServoPin output LOW level
    GPIO.setup(tiltServo2Pin, GPIO.OUT)  # Set tiltServo2Pin to OUTPUT mode
    GPIO.output(tiltServo2Pin, GPIO.LOW) # Make tiltServo2Pin output LOW level

    p = GPIO.PWM(panServoPin, 50)    # set Frequece to 50Hz
    p.start(0)                       # Set initial Duty Cycle to 0
    t = GPIO.PWM(tiltServoPin, 50)   # set Frequece to 50Hz
    t.start(0)                       # Set initial Duty Cycle to 0
    t2 = GPIO.PWM(tiltServo2Pin, 50) # set Frequece to 50Hz
    t2.start(0)                      # Set initial Duty Cycle to 0

def alarm_setup():
    GPIO.setmode(GPIO.BCM)               # use PHYSICAL GPIO Numbering
    GPIO.setwarnings(False)              # Remove "already in use warning"
    global Enb,In3,In4, pwm
    Enb,In3,In4 = 5,6,16
    GPIO.setup(Enb,GPIO.OUT)
    GPIO.setup(In3,GPIO.OUT)
    GPIO.setup(In4,GPIO.OUT)
    pwm = GPIO.PWM(Enb,200)
    pwm.start(0)

def alarm():
    GPIO.output(In3,GPIO.HIGH)
    GPIO.output(In4,GPIO.LOW)
    pwm.ChangeDutyCycle(50)    # Operate at 50% of Ena at 100Hz

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
    t2.ChangeDutyCycle(map(angle,0,180,SERVO_MAX_DUTY,SERVO_MIN_DUTY)) # map the angle to duty cycle and output it

def loop():
    fireDet = False
    xServo = 0
    yServo = 80
    y2Servo = 80

    pServoWrite(xServo)
    tServoWrite(yServo)
    t2ServoWrite(y2Servo)
    time.sleep(0.5)

    panCamera(fireDet)

def panCamera(fireDet):
    print ('Camera Searching For Fire')
    while(not fireDet):
        for i in range(0, 180, 3):
            data = sp.read(1)[0]
            xServo = i
            pServoWrite(xServo)
            if(not(data == 244)):
                fireDet = True
                servoControl(xServo, 90, 90, 0)
                break

        if(fireDet):
            continue

        for i in range(180, 0, -3):
            data = sp.read(1)[0]
            xServo = i
            pServoWrite(xServo)
            if(not(data == 244)):
                fireDet = True
                servoControl(xServo, 90, 90, 0)
                break
            #time.sleep(0.01)

def servoControl(xServo, yServo, y2Servo, emailNum):
    print ('Fire Detected')
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
        yServo = yServo - pidy(yErr) #yServo - (yPGain*yErr)
        y2Servo = y2Servo - pidy(yErr) #y2Servo - (yPGain*yErr)

        # Print current angle of pan and tilt servomotors from horizontal
#        print ("pan servo: %2f, tilt servo: %2f" % (xServo, 90 - yServo))

    # Print Pan/Tilt Error
        print (xErr)

        # Correct for servomotors going beyond angle bounds
        if(yServo > 180 and y2Servo < 0):
            yServo = 180
            y2Servo = 0
        elif(yServo < 0 and y2Servo > 180):
            yServo = 0
            y2Servo = 180

        if(xServo > 180):
            xServo = 180
        elif(xServo < 0):
            xServo = 0

        # Move servomotors to angle
        pServoWrite(xServo)
        tServoWrite(yServo)
        t2ServoWrite(y2Servo)
#        print ("Fire is % 3f feet away" %(read_lidar()/12.0))

        # If camera is centered, then email snapshot of photo
        if((xErr <= 1 and xErr >= -1) and (yErr <= 1 and yErr >= -1) and emailNum == 0):
            emailPhoto(xServo, yServo, y2Servo)

def destroy():
    p.stop()
    t.stop()
    t2.stop()
    GPIO.cleanup() # changes the used GPIO pins to input
    sp.close()     # Closes the serial port

def emailPhoto(xServo, yServo, y2Servo):
     print ('Emailing Photo of Fire')
     sp.flush()
     i = 0
     while (True):
         #sp.write("snap")
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
              alarm()
              servoControl(xServo, yServo, y2Servo, 1)

def read_lidar():
    while (True):
        try:
            # We print tuples so you can plot with Mu Plotter
            lidar_inches = (sensor.distance - 10)/2.54 # Distance in inches. Please note the -10 is for correcting the sensor measurement in cm.
        except RuntimeError as e:
            # If we get a reading error, just print it and keep truckin'
            lidar_inches = e

        print(lidar_inches)
#    return lidar_inches

if __name__ == '__main__':     # Program entrance
    print ('Fire detection system is starting...')
    alarm_setup()
    servo_setup()
    setupEmail()
    openmv_reset()
    try:
        loop()
    except KeyboardInterrupt:  # Press ctrl-c to end the program.
        destroy(
