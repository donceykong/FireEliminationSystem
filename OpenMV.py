# Doncey Albin, (3/4/2021)
# This is to test sending pixel error to RaspberriPi for processing...
# Ultimately, it may be best to do all controls from the RaspberryPi, while the
# OpenMV camera just provides data on the fire's wherabouts from the camera frame.

import time
from pyb import Servo, USB_VCP
import sensor, time, math, pyb, image, ustruct

def setup():
    # Color Tracking Thresholds (Grayscale Min, Grayscale Max)
    global threshold_list
    threshold_list = [(200, 255)]

    # Set the target temp range here
    min_temp_in_celsius = 0.0
    max_temp_in_celsius = 64.0

    print("Resetting Lepton...")
    # These settings are applied on reset
    sensor.reset()
    sensor.ioctl(sensor.IOCTL_LEPTON_SET_MEASUREMENT_MODE, True)
    sensor.ioctl(sensor.IOCTL_LEPTON_SET_MEASUREMENT_RANGE, min_temp_in_celsius, max_temp_in_celsius)
    print("Lepton Res (%dx%d)" % (sensor.ioctl(sensor.IOCTL_LEPTON_GET_WIDTH),
                              sensor.ioctl(sensor.IOCTL_LEPTON_GET_HEIGHT)))
    print("Radiometry Available: " + ("Yes" if sensor.ioctl(sensor.IOCTL_LEPTON_GET_RADIOMETRY) else "No"))

    # FPS clock
    global clock
    clock = time.clock()


# Only blobs that with more pixels than "pixel_threshold" and more area than "area_threshold" are
# returned by "find_blobs" below. Change "pixels_threshold" and "area_threshold" if you change the
# camera resolution. "merge=True" merges all overlapping blobs in the image.
def smallPic():
    sensor.set_pixformat(sensor.GRAYSCALE)
    sensor.set_framesize(sensor.QQQQVGA)

def bigPic():
    sensor.set_pixformat(sensor.GRAYSCALE)
    sensor.set_framesize(sensor.QQVGA)

def main():
    # usb object
    usb = USB_VCP()
    i = 0
    j = 0
    emailNum = 0
    while(True):
        clock.tick()
        img = sensor.snapshot(pixformat=sensor.GRAYSCALE)
        blobs = img.find_blobs(threshold_list, pixels_threshold=0, area_threshold=0, merge=True)
        cmd = usb.recv(4, timeout = 1)
        
        if (cmd == b'snap'):
            i = 0
            emailNum = 0
        
        if (i == 0):
            usb.send(244)

        if (blobs):
            i = 1
            if (i == 1):
                fire_blob = max(blobs, key = lambda x: x.density())
                img.draw_rectangle(fire_blob.rect())
                img.draw_cross(fire_blob.cx(), fire_blob.cy())

                # For pan control
                xPos = fire_blob.cx()
                xErr = 20 - xPos

                # For tilt control
                yPos = fire_blob.cy()
                yErr = 15 - yPos

                #img.draw_string(fire_blob.x(), fire_blob.y() - 10, "Pan Error: %.2f pixels" % xPosErr, mono_space=False)
                usb.send(xPos)
                usb.send(yPos)

                if((xErr <= 1 and xErr >= -1) and (yErr <= 1 and yErr >= -1) and emailNum == 0):
                    bigPic() # Convert to better resolution for email photo
                    img2 = sensor.snapshot(pixformat=sensor.GRAYSCALE)
                    blobs = img2.find_blobs(threshold_list, pixels_threshold=0, area_threshold=0, merge=True)
                    fire_blob = max(blobs, key = lambda x: x.density())
                    img2.draw_rectangle(fire_blob.rect())
                    img2.draw_cross(fire_blob.cx(), fire_blob.cy())
                    img2.to_rainbow(color_palette=sensor.PALETTE_IRONBOW) # color it

                    imgUSB = img2.compress()
                    usb.send(ustruct.pack("<L", imgUSB.size()))
                    usb.send(imgUSB)
                    emailNum = 1
                    smallPic() # Convert back to low-res imaging for control

# Program Enterance
setup()
smallPic()
main()
