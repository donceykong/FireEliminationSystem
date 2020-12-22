# Doncey Albin
# 10/27/2020
# This script was made to pan and tilt the OpenMV camera to a surface with the temperature threshold of interest.

from pyb import Servo
import sensor, image, time, math, pyb

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

s1.pulse_width(xPosToPulse) # 0deg: 500, +45deg: +500 pulses, 180deg: 2500
s2.pulse_width(yPosToPulse) # 0deg: 500, +45deg: +500 pulses, 180deg: 2500
time.sleep(4) # sleep for 3 seconds before panning to the object

# initialize increment
i = 0;

# Initialize Err lists for pan and tilt
xErrList = []
yErrList = []

while(True):
    time = pyb.millis()
    clock.tick()
    img = sensor.snapshot()
    for blob in img.find_blobs(threshold_list, pixels_threshold=100, area_threshold=100, merge=True):
        stats = img.get_statistics(thresholds=threshold_list, roi=blob.rect())
        img.draw_rectangle(blob.rect())
        img.draw_cross(blob.cx(), blob.cy())
        #img.draw_string(blob.x(), blob.y() - 10, "%.2f C" % map_g_to_temp(stats.mean()), mono_space=False)
        img.to_rainbow(color_palette=sensor.PALETTE_IRONBOW) # color it
        #print("FPS %f - Lepton Temp: %f C" % (clock.fps(), sensor.ioctl(sensor.IOCTL_LEPTON_GET_FPA_TEMPERATURE)))

        # increment the increment
        i = i + 1

        # For pan control
        xPos = blob.cx()
        xPosErr = 80 - xPos
        xErr_PGain = 0.6

        # For tilt control
        yPos = blob.cy()
        yPosErr = 60 - yPos
        yErr_PGain = 0.6

        if xPos != 80:
            xPosToPulse = int(xPosToPulse + xErr_PGain*xPosErr)
            s1.pulse_width(xPosToPulse) # 0deg: 500, +45deg: +500 pulses, 180deg: 2500

        if yPos != 60:
            yPosToPulse = int(yPosToPulse - yErr_PGain*yPosErr)
            s2.pulse_width(yPosToPulse) # 0deg: 500, +45deg: +500 pulses, 180deg: 250

        img.draw_string(blob.x(), blob.y() - 20, "Pan Error: %.2f pixels" % xPosErr, mono_space=False)
        img.draw_string(blob.x(), blob.y() - 10, "Tilt Error: %.2f pixels" % yPosErr, mono_space=False)
        print(xPosErr)
        print(yPosErr)
