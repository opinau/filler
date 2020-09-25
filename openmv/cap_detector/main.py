#*****************************************
# botele cap detector
# created 6. 2020.
# Project OPINAU-FILLER-CV
#*****************************************
#
# IMPORTANT!!
# rename script to main.py and copy to camera root
# after reboot camera should start with this script
#
# please note, SD card is required which stores templates for matching
# place template on SD card root :
# cap_ex_left2.pgm,cap_ex_right2.pgm, nocap_ex_left.pgm, nocap_ex_right.pgm
# user can replace provided tempaltes with their own
# can be edited in graphic tools (eg. Photoshop)
# store it as .pgm grayscale file, keep it small (like 50 x 100 px)
#
# Please note that this code runs on microcontroller, not on computer, larger template may significantly reduce detection speed
#
#############################################################
# System detects bottle, and perform analysis if cap is present
# reports status over UART (on pin4)
# message example: <bottle_counter,detection_counter,status,angle,str_message>
# available status:
# 0 no bottle ->system detected something that is not bottle
# 1 cap  -> bottle with cap is detected
# 2 no cap ->bottle with not cap detected
# 3 bad cap ->bottle with bad cap detected
# 4 bad roi -> only part of bottle is detected, please reposition porximity sensor
#
# bottle_counter counts detected bottle
# detection_counter count how many objects proximity sensor detected (may not be == bottle_counter)
#
##############################################################
#
# please use 3.3V enabled device on receiving side, if required logic level convertor is an option
# for forwarding commands on Arduino based boards (NANO - ATMEGA328P) logic convertion is not required
#
# Hardware is based on OpenMV H7 camera
# Proximity sensor on PIN1 (please note that device is usign reverse logic, LOW when object is present)
# MOSFET / LED / difuser on PIN3
# UART TX on PIN4
# Proximity sensor does not use enable pin (in any other form) as "slow" startupt time may reduce systems refresh rate
#
# IMPORTANT!!! Remove jumper when providing power to camera from USB, return it for autonomus operation!!!
# Camera operates with 3.6V provided over voltage / current regulator
#
# For additional info please consult provided documentation
############################################################

#some imports libraries etc.
from pyb import Pin #use hardware inputs / outputs
from pyb import LED #use bult-in LED
import time  #use timers
import math  #use trigonometry functions

import sensor, image, pyb # for using camera and basic image processing functions
from image import SEARCH_EX, SEARCH_DS  #important for template matching, defines an exhaustive search for the image
from pyb import UART  #enables use of hardware serial (UART)


#HARDWARE INPUTS AND OUTPUTS
pin1 = Pin('P1', Pin.IN, Pin.PULL_UP)  #input, to proximity sensor
pin3 = Pin('P3', Pin.OUT_PP, Pin.PULL_NONE) #output, to LED diffuser (MOSFET)
#Please note that TX UART is connected to P4 /PIN4

#############################################################
#some definitions, global variables etc.
# User can change this values!

#USER changable:
low_threshold = (0, 150) #defining threshold limit for image binarisation (set to 150 as image is allready binary-grayscale)

max_cap_angle=5 #max cap angle, if angle is larger that reported sytem reports BAD CAP
#Feel free to change max_cap_angle, 5° is obviously to large, and is set primarly for debugging purposes

user_exposure_time=400 #camera exposure time, feel free to change. 400 is default value for tested setup
# if backliht cannt povide suficient brightness, increaase  user_exposure_time
# Tested system is lighted by 4 x 0.5W LED-s placed on difusser corners
# any other type of backlight illumination should be suffucuent (xenon lamp, continious backligjht etc.)

####### #please don't chane anythin pass this line
last_proximity_state=1 #remembering last state from proximity sensor, set to HIGH /1 (reverse logic)

#Use bilt in LED, usefull for debugging purposes
# please note that additional light source reflected from polished bottle surface can confuse camera
red_led   = LED(1) # use only for debugging / camera status, see previous line / not really used in code

sensor.reset() # Initialize the camera sensor
sensor.set_pixformat(sensor.GRAYSCALE) # set camera format to grayscale (color not important in this example)
sensor.set_framesize(sensor.QVGA) # set camera resolution to QVGA 320 x 240
sensor.set_auto_exposure(False, exposure_us=user_exposure_time) # set exposure time, user changable (user_exposure_time variable)
sensor.set_auto_gain(False,gain_db=10) #set camera gain, keep it as this value is optimised for given light source and exposure time

#these setting are manually set in order to prevent camera to change it during use (no automatic setting in machine vision!)
sensor.set_brightness(2) #set camera brightness
sensor.set_contrast(3) #set camera contrast
sensor.set_saturation(0) #set camera saturation


###################################################
### input tempate images
# Please use small (eg 50 x 100 px or smaller) images in PGM format, grayscale
# place in camera root (stored microSD card)
# note that copy_to_fb = True is not used, since extra fb is required for morphological operations
# newer / more advanced cameras like M7plus may have sufficient memory to store multiple frame buffers and further accelerate code
# this tempalte is compatible to most standard beer bottles available in stores
# if different beer bottle is used, please create your own template that should match provided template

template_cap1 = image.Image("cap_ex_left2.pgm")  #template for left side of a cap
template_cap2 = image.Image("cap_ex_right2.pgm") #template for right side of a cap

template_nocap1 = image.Image("nocap_ex_left.pgm")  #template for left side of a bottle, no cap
template_nocap2 = image.Image("nocap_ex_right.pgm") #template for right side of a bottle, no cap

#extra frame buffer allocation, size and format is matching sensor / camera
# please note that with QVGA and grayscale M7 camera can only store 1 additional frame buffer
extra_fb = sensor.alloc_extra_fb(sensor.width(), sensor.height(), sensor.GRAYSCALE)

#defining Serial / UART, on PIN4, 9600 baudrate
#as on receiving side is Arduno Nano with softserial, faster baudrate may result in instability
uart = UART(3, 9600)  #UART pin 4, 9600 baudrate

sensor.skip_frames(time = 2000) #small 2 seconds pause, so camera can update its settings

## global variables
botte_counter=0 #bottle counter, initially set to 0
detection_counter=0 #dettection counter (proximity sensor), initially set to 0


## end of header, definitions etc.


###############################################################

#here goes the main loop

while(True):  # run it unitil power run out

    proximity_sensor_state=pin1.value() #read state from the proximity sensor

    #detection status flag, default is 0 (no detection, poximity detcted something which is not bottle)
    #status is changed to another value when "something" is detected
    #please consult code header or documentation
    detection_status=0 #default value is 0

    #check sensor status, and decide if image analysis is reqired
    execute_analysis=0 #flag that states if image analysis should be performed, default value is 0 / FLASE
    # please note that detecting proximity sensor change is not solved using extrenal interrupt
    # proximity sensor may fire few HIGH /LOW states when object is near border position for detection
    # slow analysis of image (~300ms) is acting like debouncig of input signal

    #check if thete is any change on proximity sensor, note inverse logic!
    # if there is object present for a long time, it only analyses it one time (when object enters camera ROI)
    if  proximity_sensor_state==0 and last_proximity_state==1: #new state is 0, and last was 1
        execute_analysis=1  # change flag execute_analysis to 1

    last_proximity_state=proximity_sensor_state #remember last proximity sensor state


    #this part is executed only if new object is present, execute_analysis is set to 1
    if  execute_analysis==1: # check if flag execute_analysis is set to 1

        detection_counter+=1 #increment detection counter

        execute_analysis=0 # reset flag execute_analysis to 0,

        pin3.value(1) #set PIN3 to high, fires up LED flash
        time.sleep(10) #small delay, allow LED to reach optimal brightness

        img = sensor.snapshot(1) #take image annd sore it in img object
                                 #replace with img = sensor.snapshot(2) which takes 2 images if there is problem with stability

        time.sleep(2) #small delay before turning of LED
        pin3.value(0) #turn OFF led flasher

        #here goes image processing
        img.binary([low_threshold]) #thresholding original image,low_threshold is set in code header
        img.lens_corr(1.8) #lense distorsions are corrected, parameters for 2.8 mm lense (change if different lense is used)

        #some morphological operations, this basically creates edges of detected object
        #this code is highly optimised, and performs better that inluded find edges fucntions
        img.close(2)            #binary image closing, reparing some artifactis on image
        extra_fb.replace(img)   #placing current image in extra frame buffer
        img.invert()            #inverting original binary image (0 to 1 and 1 to 0)
        extra_fb.dilate(2)      #binary image in extra_fb is dilated (2pix size of edge is removed)
        img.b_and(extra_fb)     #AND operation extracts edges from original image
        img.dilate(2)           #edges are dilated for 2 pixels (more robust template matching)


        threshold = [192, 255]  #threshold for binary image, as all pixels are 0 or 255 192 should do the trick

        ###### object analysis
        # in this part system detect if any blob is detected on image
        blob_pos_x=20  #blob starting position, x axis, default value is 10
        blob_pos_y=10  #blob starting position, y axis, default value is 10

        bottle_cap_angle=0 #bottle cap angle, starting value is 0
        #please note that resulting angle is not calculated with high precision (2-pixels precision is implemented)

        bad_roi=0 #flag that detects bad roi -  object is not inside camera ROI, to fast object  etc.

        # search for any blob in image, set taht min area size is 100 (area_threshold=100), containing minimum 100 pixels (pixels_threshold=100)
        # merge nearby blobs (merge=True) if discontinuity in edge is present (10 pixels size for margin=10)
        # object blob is returned if any blob is found
        for blob in img.find_blobs([threshold], pixels_threshold=100, area_threshold=100, merge=True, margin=10):
            #img.draw_rectangle(blob.rect())
            #img.draw_cross(blob.cx(), blob.cy())
            #print("blob at", blob.rect())
            blob_pos_x=blob.x()-3  #blob starting position is deducted for 3 pix, so blob can containg area outside edge
            blob_pos_y=blob.y()-3  #same thing for y direction /axis
            detection_status=3     #change status to 3, ad some blob is detected!
                                   # if not blob is deetcted status remains 0 (no object detected)

            #check if blob start position is outside camera image, not possible but we do not want any errors
            if blob_pos_x<1:
                blob_pos_x=1;
            if blob_pos_y<1:
                blob_pos_y=1;


            #check blob size
            #usual blob size for bottle detection is 250 x 250, much lower values should result in error
            if blob.w()<200 and blob.h()<180: #if blob size is less than 200 x 180 pix, camera cannot see whole bottle

                bad_roi=1          #bad_roi flag is set to 1
                detection_status=4 #report bad roi, user should reposition proximity sensor

        #continue with analysis ony if blob size is inside defined parameters!
        if bad_roi==0:  #continue only if flag bad_roi is set to 0


            #crop input image to size 260 x 180, smaller image faster execution!
            # 260 x 180 image allows template matching algorithm to run smoothly
            # copy_to_fb parameter enables operation inside framebuffer (much faster than normal crop/copy)
            img.crop(roi = (blob_pos_x,blob_pos_y,260,180),copy_to_fb = True)
            #print(img.width(),'www ',img.height())

            #check if cropped image is size larger than template, prevent img.find_template error!
            if img.width()>100 and img.height()>100: #usual template size is 50 x 100 pix

                # search for tempalte location, object r is returned
                # search for template template_cap1, matching value is 0.6, decreese it for better detection (may result in false positives)
                # set size is set to 2 pixels (faster search), search type is search=SEARCH_EX wich performs search on whole area
                # search is performed on area defined by ROI roi=(1, 1, img.width(), img.height()) , basically whole cropped image
                r = img.find_template(template_cap1, 0.60, step=2, search=SEARCH_EX, roi=(1, 1, img.width(), img.height()))
                # if left template is matched, then search for right one, else do not seach any further
                if r:
                    # same setup as before for find_template, result is stored in r2 object
                    r2 = img.find_template(template_cap2, 0.60, step=2, search=SEARCH_EX, roi=(1, 1, img.width(), img.height()))
                    if r2:  # if secod template is found

                        img.draw_string(100, 20, "CAP DETECTED") #write cap is detected on framebuffer image, does not do enything but looks cool
                        #calculates cap angle
                        #please not that angle accuaricy is not sho hight, as this code operates in 2pixels precision
                        temp_angle=math.atan2(r[1]-r2[1],r[0]-r2[0] ) #calculates angle of deetcted cap
                        bottle_cap_angle=180-math.degrees(temp_angle)

                        # check if angle is higher than 180 deg (basically negative values of angle)
                        if bottle_cap_angle>180:
                            bottle_cap_angle=360-bottle_cap_angle # resulting cap angle is +/- few degrees
                        detection_status=1 #set detection status to 1, as cap is detected (default is 0)
                        botte_counter+= 1  # increment bottle counter by one

                #search for another template (no cap)
                #search options as before for find_template
                r = img.find_template(template_nocap1, 0.60, step=2, search=SEARCH_EX, roi=(1, 1, img.width(), img.height()))
                if r: # if left nocap template is found
                    #serch if there is right nocap template on image
                    r2 = img.find_template(template_nocap2, 0.60, step=2, search=SEARCH_EX, roi=(1, 1, img.width(), img.height()))
                    if r2:

                        img.draw_string(100, 20, "NO CAP DETECTED") #write no cap cap detected on framebuffer image, does not do enything but looks cool
                        detection_status=2  # set detection status to 2, as no_cap is detected (default is 0)
                        botte_counter+= 1   # increment bottle counter by one

                #check if bottle cap angle is higher than defined number in max_cap_angle
                if abs(bottle_cap_angle)>max_cap_angle :
                  detection_status=3  #if angle is to large, set detection status to 3 (bad cap), cap is detected but angle is bad, possible pressure drop in capping machine

                ## please note that if status is not changed to 1 (cap) 2 (nocap) or 3 (bad cap), status reamins default 0 (no object)



    ##################################
    # reporting over Serial (UART)
    # UART message format
    # <bottle_counter,detection_counter,status,angle,str_message>
    # bottle_counter - current number of detected bottle
    # detection_counter- current proximity senosr event
    # status:
    #    0 no bottle/ no object
    #    1 cap
    #    2 no cap
    #    3 bad cap (bad cap angle)
    #    4 bad roi (reposition proximity sensor or object is too fast)


        #report status 0 - no bottle/ no object
        if detection_status==0:
                #create message string, see format
                message=("<")+str(botte_counter)+","+str(detection_counter)+", , ,STATUS_OK>\n"
                #print message on terminal
                print(message)
                #send same message over UART (SERIAL)
                uart.write(message)

        #report status 1 - cap
        if detection_status==1:
                #create message string, see format
                message=("<")+str(botte_counter)+","+str(detection_counter)+",1,"+str(bottle_cap_angle)+" ,STATUS_OK>\n"
                #print message on terminal
                print(message)
                #send same message over UART (SERIAL)
                uart.write(message)

        #report status 2 - no cap
        if detection_status==2:
                #create message string, see format
                #message="Bottle no=" +str(botte_counter)+" NO CAP detected\n"
                message=("<")+str(botte_counter)+","+str(detection_counter)+",2, ,STATUS_OK>\n"
                #print message on terminal
                print(message)
                #send same message over UART (SERIAL)
                uart.write(message)

        #report status 3- bad cap (bad cap angle)
        if detection_status==3:
            #create message string, see format
            message=("<")+str(botte_counter)+","+str(detection_counter)+",3, ,STATUS_OK>\n"
            #print message on terminal
            print(message)
            #send same message over UART (SERIAL)
            uart.write(message)

        #report status 4- bad roi (reposition proximity sensor or object is too fast)
        if detection_status==4:
            #create message string, see format
            message=("<")+str(botte_counter)+","+str(detection_counter)+",4, ,STATUS_OK>\n"
            print(message)
            #print message on terminal
            uart.write(message)
            #send same message over UART (SERIAL)


        #extra line added in terminal,
        print(" ")

    #else:
        #put something here if execute_analysis==0

### end of code







