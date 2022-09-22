#!/usr/bin/env python

import time

import sys

import Adafruit_GPIO.SPI as SPI
import Adafruit_SSD1306

from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont

import subprocess

#ROS
import rospy
from std_msgs.msg import String


# Raspberry Pi pin configuration:
RST = None     # on the PiOLED this pin isn't used
# Note the following are only used with SPI:
DC = 23
SPI_PORT = 0
SPI_DEVICE = 0

#disp = Adafruit_SSD1306.SSD1306_128_32(rst=RST)
disp = Adafruit_SSD1306.SSD1306_128_64(rst=RST)

# Initialize library.
disp.begin()

# Clear display.
disp.clear()
disp.display()

# Create blank image for drawing.
# Make sure to create image with mode '1' for 1-bit color.
width = disp.width
height = disp.height
image = Image.new('1', (width, height))

# Get drawing object to draw on image.
draw = ImageDraw.Draw(image)

# Draw some shapes.
# First define some constants to allow easy resizing of shapes.
padding = -2
top = padding
bottom = height-padding
# Move left to right keeping track of the current x position for drawing shapes.
x = 0

# Load default font.
font = ImageFont.load_default()

# Alternatively load a TTF font.  Make sure the .ttf font file is in the same directory as the python script!
# Some other nice fonts to try: http://www.dafont.com/bitmap.php

update_display_interval = 1.0 #sec

global OLEDtext
OLEDtext = ""
OLEDtext_5 = OLEDtext_6 = OLEDtext_7 = OLEDtext_8 = ""

def display(data=OLEDtext):
    # Draw a black filled box to clear the image.
    draw.rectangle((0,0,width,height), outline=0, fill=0)

    cmd = "hostname -I | cut -d\' \' -f1"
    IP = subprocess.check_output(cmd, shell = True )
    cmd = "top -bn1 | grep load | awk '{printf \"CPU Load: %.2f\", $(NF-2)}'"
    CPU = subprocess.check_output(cmd, shell = True )
    cmd = "free -m | awk 'NR==2{printf \"Mem: %s/%sMB %.2f%%\", $3,$2,$3*100/$2 }'"
    MemUsage = subprocess.check_output(cmd, shell = True )
    cmd = "df -h | awk '$NF==\"/\"{printf \"Disk: %d/%dGB %s\", $3,$2,$5}'"
    Disk = subprocess.check_output(cmd, shell = True )

    IP = IP.decode("utf-8")
    CPU = CPU.decode("utf-8")
    MemUsage = MemUsage.decode("utf-8")
    Disk = Disk.decode("utf-8")

    OLEDtext_1 = str(IP)
    OLEDtext_2 = str(CPU)
    OLEDtext_3 = str(MemUsage)
    OLEDtext_4 = str(Disk)

    global lineNr, OLEDtext
    try:
        OLEDtext_raw = data.data
        [lineNr, OLEDtext] = OLEDtext_raw.split("_")
        lineNr = int(lineNr.replace("_", ""))

        if (lineNr == 5):
            global OLEDtext_5
            OLEDtext_5 = str(OLEDtext)
        elif (lineNr == 6):
            global OLEDtext_6
            OLEDtext_6 = str(OLEDtext)
        elif (lineNr == 7):
            global OLEDtext_7
            OLEDtext_7 = str(OLEDtext)
        elif (lineNr == 8):
            global OLEDtext_8
            OLEDtext_8 = str(OLEDtext)
    except:
        OLEDtext = data

    draw.text((x, top),    OLEDtext_1, font=font, fill=255)
    draw.text((x, top+8),  OLEDtext_2, font=font, fill=255)
    draw.text((x, top+16), OLEDtext_3, font=font, fill=255)
    draw.text((x, top+24), OLEDtext_4, font=font, fill=255)
    draw.text((x, top+32), OLEDtext_5, font=font, fill=255)
    draw.text((x, top+40), OLEDtext_6, font=font, fill=255)
    draw.text((x, top+48), OLEDtext_7, font=font, fill=255)
    draw.text((x, top+56), OLEDtext_8, font=font, fill=255)

    disp.image(image)
    disp.display()

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/OLED/CmdsetText", String, display)
    rospy.loginfo("Starting subscribing to text to oled")

if __name__ == '__main__':

    listener()

    starttime = time.time()
    while True:
        display(OLEDtext)
        time.sleep(update_display_interval - ((time.time() - starttime) % update_display_interval))

        # spin() simply keeps python from exiting until this node is stopped
        #rospy.spin()
