# import all necessary components
import smtplib
from email.mime.text import MIMEText
from email.mime.image import MIMEImage
from email.mime.multipart import MIMEMultipart

import sys, serial, struct
port = '/dev/ttyACM0'

sender_email = "donceyalbin@gmail.com"
receiver_email = "donceyalbin@gmail.com"
password = "Donthackme2MF33"

# Create a multipart message and set headers
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

port = '/dev/ttyACM0'
sp = serial.Serial(port, baudrate=115200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
             xonxoff=False, rtscts=False, stopbits=serial.STOPBITS_ONE, timeout=None, dsrdtr=True)

i = 0
while(True):
     sp.setDTR(True) # dsrdtr is ignored on Windows.
     size = struct.unpack('<L', sp.read(4))[0]
     img = sp.read(size)

     if (img > 0):
          i = i + 1
          print("fire detected " + str(i) + " times.")
          with open("img" + str(i) + ".jpg", "w") as f:
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
          
#else:
sp.close()


