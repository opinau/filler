#*****************************************
# high precission bottle positiion sensor
# created 6. 2020.
# Project OPINAU-FILLER-CV
#*****************************************
#
# IMPORTANT!!
# rename script to main.py and copy to camera root
# after reboot camera should start with this script
#
# Please note that this code runs on microcontroller, not on computer, larger template may significantly reduce detection speed
#
#############################################################
# System detects flashing laser line and calculates bottle center
# reports status over UART (on pin4)
# message example: <position_x,position_y, timestamp,str_message>
# position_x and position_y in mm
# timestamp in ms
# str_message is extra status, not used at the moment (only STATUS_OK is reported)
#
##############################################################
#
# please use 3.3V enabled device on receiving side, if required logic level convertor is an option
# for forwarding commands on Arduino based boards (NANO - ATMEGA328P) logic convertion is not required
#
# Hardware is based on OpenMV H7 camera
# Laser is contreooled over PIN1
# UART TX on PIN4
#
# IMPORTANT!!! Remove jumper when providing power to camera from USB, return it for autonomus operation!!!
# Camera operates with 3.6V provided over voltage / current regulator
#
# For additional info please consult provided documentation
############################################################

#some imports libraries etc.

import sensor, image, time # for using camera and basic image processing functions
from pyb import Pin #use hardware inputs / outputs
from pyb import UART  #enables use of hardware serial (UART)
import math #use trigonometry functions

#importing lookup tables, placed inside lut_table.py script
from lut_table import lut_distance


#HARDWARE INPUTS AND OUTPUTS
uart = UART(3, 9600)  #UART pin 4, 9600 baudrate
pin1 = Pin('P1', Pin.OUT_PP, Pin.PULL_NONE)  #output, to laser (MOSFET)


# gain setting
GAIN_SCALE = 1
polozaj_linije_ypos=100 #tu se zapisuje polozaj vertikalne linije, prepraviti kalibracijom

#USER changable:
low_threshold = (30, 255) #defining threshold limit for image binarisation

#enter bottle diameter in mm, default is 67 mm
bottle_diameter=67 #bottle diameter in mm


#define min spot size (reflected laser line), optimised for 15-30 cm distance
num_pixels_threshold=15 #minimal number of pixles detected, default is 25
num_area_threshold=20  #minimal area covered by blob , default is 30

#lens distortion correction parameters
#please consult lense_correction script (lense_correction.py)
z_rotation=0
lense_flenght=1.55 #defualt is 1.55
lense_x_corr=0.04  # lense center shifted in x axis
lense_y_corr=0.12  #lense center shifted in y axis

lut_table = image.Image("LUT.bmp") #LUT for x pos, for simplicity stored as a bitmap image
# LUT table is built for bottle base
# if some other part of bottle (like bottle neck) another LUT table should be built
# in some scenarios reflective stickers can cause problems and confuse MV system
# bottleneck should be witouth any sticker...

user_exposure_time=500 #camera exposure time, feel free to change. 900 is default value for tested setup


sensor.reset() # Initialize the camera sensor
sensor.set_pixformat(sensor.GRAYSCALE) # set camera format to grayscale (color not important in this example)
sensor.set_framesize(sensor.QVGA) # set camera resolution to QVGA 320 x 240
sensor.set_auto_whitebal(False) # Turn off white balance
sensor.set_auto_exposure(False, exposure_us=user_exposure_time) # set exposure time, user changable (user_exposure_time variable)

#these setting are manually set in order to prevent camera to change it during use (no automatic setting in machine vision!)
sensor.set_brightness(0) #set camera brightness
sensor.set_contrast(2) #set camera contrast
sensor.set_saturation(0) #set camera saturation


# Print out the initial gain for comparison.
print("Initial gain == %f db" % sensor.get_gain_db())
sensor.skip_frames(time = 2000)     #small 2 seconds pause, so camera can update its settings

# calculating sensor gain
# user can change GAIN_SCALE factor in order to obtain more bright image
current_gain_in_decibels = sensor.get_gain_db() #current sensor gain
print("Current Gain == %f db" % current_gain_in_decibels) #reporting current gain
sensor.set_auto_gain(False,gain_db = current_gain_in_decibels * GAIN_SCALE) #new gain
print("New gain == %f db" % sensor.get_gain_db()) #reporting new sensor gain
sensor.skip_frames(time = 2000)     #small 2 seconds pause, so camera can update its settings
#please note that this secod skip_frames is not an error


#extra frame buffer allocation, size and format is matching sensor / camera
# please note that with QVGA and grayscale H7 camera can only store 1 additional frame buffer
extra_fb = sensor.alloc_extra_fb(sensor.width(), sensor.height(), sensor.GRAYSCALE)

clock = time.clock()   # Create a clock object to track the FPS.

## end of header, definitions etc.

###############################################################

#here goes the main loop

while(True):
    clock.tick() # Track elapsed milliseconds between snapshots().

    #taking first image, and storing in extra frame buffer
    extra_fb.replace(sensor.snapshot())

    #this part starts laser projector and takes second image
    time.sleep(2) #small pause before taking another image
    pin1.value(1)  #powering on laser projector
    time.sleep(2) #small pause to allow laser to start
    img = sensor.snapshot() # Take a picture and return the image
    pin1.value(0) #powering off laser projector
    time.sleep(2) #small pause before image processing
    # both images are stored in frame buffer, laser turns off

    ## here goes image processing part
    #image differencing, same scene with and witouth laser line
    img.difference(extra_fb)

    #calculating histograms
    #img.draw_rectangle(1, 100, 320, 140,fill=True, color=(0)) #draw black rectangle in lower part
    #img.histeq(adaptive=True, clip_limit=3)
    #get histogram statistics
    stats=img.get_statistics()
    hist=img.get_histogram()

    #starting blob postions (initially placed on image edge)
    top_blob_cy=240
    top_blob_cx=-1

    #undistort image with provided parameters
    img.rotation_corr(x_rotation =0,y_rotation =0,z_rotation =z_rotation) #repair rotation
    img.lens_corr(lense_flenght,x_corr=lense_x_corr, y_corr=lense_y_corr) #repair distortion

    #definira pomak kamere u mm y smjer
    pomak_kamere_mm=60

    #search for blobs, brighthest object in image (reflected lasre line, if found)
    #for blob in img.find_blobs([(hist.get_percentile(0.997).l_value(), 255)],roi=(1, 1, 320, polozaj_linije_ypos), pixels_threshold=num_pixels_threshold, area_threshold=num_pixels_threshold, merge=True, margin=2):

    for blob in img.find_blobs([(hist.get_percentile(0.995).l_value(), 255)],roi=(1, 1, 320, polozaj_linije_ypos), pixels_threshold=num_pixels_threshold, area_threshold=num_pixels_threshold, merge=True, margin=2):
        img.draw_rectangle(blob.rect())
        img.draw_cross(blob.cx(), blob.cy())

        if blob.cy()<top_blob_cy: #search for blob with lower y pos (nearest to camera)
            top_blob_cy=blob.cy() #update blob cy
            top_blob_cx=blob.cx() #update blob cx

    if top_blob_cy<240:  #continue only if blox is found (blob shuld be in top part of image)
        succes_oper_count=0 #count succesful steps, report pos if number is ==2
        #print(top_blob_cy,"   ",top_blob_cx) #uncoment for debugging

        if top_blob_cy in lut_distance : #check if blob y pos is in LUT table (should be values 1 to 110)
            y_pos_bottle=lut_distance [top_blob_cy ] #copy value from LUT

            #calculate x position from LUT table, please note that data are shifted 125mm
            # in orded to stoer only positive values in bitmap image
            x_pos_bottle=lut_table.get_pixel(top_blob_cx-1,top_blob_cy-1)-125


            #print some data for debugging
            print("spot position in image px: ",top_blob_cx,"py: ",top_blob_cy," spot pos mm: y=",y_pos_bottle,"x=",x_pos_bottle)

            #reporting true bottle position (takes bottle diameeter in to account)
            message="<"+str(round(x_pos_bottle))+","+str(round(y_pos_bottle+bottle_diameter/2))+","+str(time.ticks())+",STATUS_OK>\n"
            print(message)
            uart.write(message)



