#*****************************************
# camera calibration
# created 7. 2020.
# Project OPINAU-FILLER-CV
#*****************************************

# use thios code to check if camera lens is correcttly undistorted
# you can use this code for building the LOT (pixles to angle and disparity to distance)

import sensor, image, time



from pyb import Pin


ZOOM_AMOUNT = 1 # Lower zooms out - Higher zooms in.
FOV_WINDOW = 60 # Between 0 and 180. Represents the field-of-view of the scene
                # window when rotating the image in 3D space. When closer to
                # zero results in lines becoming straighter as the window
                # moves away from the image being rotated in 3D space. A large
                # value moves the window closer to the image in 3D space which
                # results in the more perspective distortion and sometimes
                # the image in 3D intersecting the scene window.

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)
clock = time.clock()

#copy & paste in another scripts
z_rotation=0
lens_flenght=1.55 #1.50
lens_x_corr=0.04 #-0.02
lens_y_corr=+0.12 #+0.03

pin1 = Pin('P1', Pin.OUT_PP, Pin.PULL_NONE)

counter=0

while(True):
    counter+=1
    clock.tick()
    if (counter % 5==0):
        pin1.value(1)
    else:
        pin1.value(0)

    img = sensor.snapshot()
    img.rotation_corr(x_rotation =0,y_rotation =0,z_rotation =z_rotation)

    img.lens_corr(lens_flenght,x_corr=lens_x_corr, y_corr=lens_y_corr)
    #img.draw_rectangle(1, 100, 320, 140,fill=True, color=(0))

    print(clock.fps())
