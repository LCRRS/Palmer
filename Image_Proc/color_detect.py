#===============================================================
#====== COLOR DETECTION - NO COMMUNICATION WITH ARDUINO ========
#===============================================================

import cv2
import numpy as np
import serial
import sys

ser = serial.Serial('/dev/ttyACM0', 9600)

center_frame = (320,240)
radius_frame = (140)
area_frame = 61575
radius_frame_max = (200)
area_frame_max = 125663

source = cv2.VideoCapture(0)

while(1):
    kernel_open = np.ones((20,20),np.uint8) # Erosion values
    kernel_close = np.ones((21,21),np.uint8) #Dilution values
    _, frame = source.read() # reads one frame at a time

    # Use this to get the resolution of the picture
    # print(frame.shape)

    # BGR to HSV conversion helps better isolate a single color
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # define range of blue color in HSV
    lower_blue = np.array([50,50,50])
    upper_blue = np.array([70,255,255])

    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)
    closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel_close)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame,frame, mask= closing)
    imgray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
    ret,thresh = cv2.threshold(imgray,1,255,0)
    contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    # Use this to see the array of the object print(contours)
    # additional methods can be used on contours in order to find the area/perimeter and the center_obj

    cv2.circle(res,center_frame,radius_frame,(255,0,0),2)
    cv2.circle(res,center_frame,radius_frame_max,(0,0,255),2)

    if len(contours) > 0:
        cnt = contours[0]
        (x,y),radius_obj = cv2.minEnclosingCircle(cnt)
        center_obj = (int(x),int(y))
        radius_obj = int(radius_obj)
        cv2.circle(res,center_obj,radius_obj,(0,255,0),-1)

        area_obj = ((radius_obj**2)*3.14159265359)

        if area_obj > area_frame:
            if area_obj > area_frame_max:
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(res,'OOPS TOO CLOSE',(30,450), font, 2,(255,0,0),2)
                distance = "0\n" # means that the object is way too close, thus needs to move further away
            else:
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(res,'WARNING',(30,450), font, 2,(0,0,255),2)
                distance = "1\n" # means safe distance: not too close but not too far
        else:
            distance = "2\n" #means that the object is still far away from the camera, thus needs to move closer

        offset_hor = (str(int(center_obj[0]) - 320)+"\n")
        offset_ver = (str(int(center_obj[1]) - 240)+"\n")
        to_be_sent = ["31415", offset_hor, offset_ver, distance]

        for i in range(len(to_be_sent)):
            ser.write(to_be_sent[i])

    cv2.imshow('res',res)
    cv2.imshow('frame',frame)
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()
