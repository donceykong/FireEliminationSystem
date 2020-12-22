# Doncey Albin, (12/7/2020)
# This is to test sending pixel error to RaspberriPi for processing...
# Ultimately, it may be best to do all controls from the RaspberryPi, while the
# OpenMV camera just provides data on the fire's wherabouts from the camera frame.

import time
from pyb import Servo, USB_VCP
import sensor, time, math, pyb, image, ustruct

# Color Tracking Thresholds (Grayscale Min, Grayscale Max)
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

sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time=5000)
clock = time.clock()

# Only blobs that with more pixels than "pixel_threshold" and more area than "area_threshold" are
# returned by "find_blobs" below. Change "pixels_threshold" and "area_threshold" if you change the
# camera resolution. "merge=True" merges all overlapping blobs in the image.

def map_g_to_temp(g):
    return ((g * (max_temp_in_celsius - min_temp_in_celsius)) / 255.0) + min_temp_in_celsius

# Set servos
s1 = Servo(1) # P7
s2 = Servo(2) # P8
xPosToPulse = 1500 #Initial x-pos of Pan Servo (Center)
yPosToPulse = 1500 #Initial y-pos of Tilt Servo (Center)

# FPS clock
clock = time.clock()

# usb object
usb = USB_VCP()

while (True):
    clock.tick()
    img = sensor.snapshot(pixformat=sensor.GRAYSCALE)
    blobs = img.find_blobs(threshold_list, pixels_threshold=100, area_threshold=100, merge=True)
    
    if (blobs):
        fire_blob = max(blobs, key = lambda x: x.density())
        img.draw_rectangle(fire_blob.rect())
        img.draw_cross(fire_blob.cx(), fire_blob.cy())
        img.to_rainbow(color_palette=sensor.PALETTE_IRONBOW) # color it

        # For pan control
        xPos = fire_blob.cx()

        # For tilt control
        yPos = fire_blob.cy()

        #img.draw_string(fire_blob.x(), fire_blob.y() - 10, "Pan Error: %.2f pixels" % xPosErr, mono_space=False)
        #imgUSB = img.compress()
        #usb.send(ustruct.pack("<L", imgUSB.size()))
        
        usb.send(xPos)
        usb.send(yPos)