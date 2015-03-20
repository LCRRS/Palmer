#===============================================================
#===============         COLOR TRACKING          ===============
#===============================================================
#====== The Color Tracking algorithm is written to      ========
#====== provide a multirotor drone real-time            ========
#====== image-processing abilities through serial       ========
#====== communication between RaspberryPi, responsible  ========
#====== for image processing and offset calculation,    ========
#====== and Arduino, serving as the primary flight      ========
#======                     controller                  ========
#===============================================================
#====== The color Detection System is written in Python ========
#====== using the OpenCV module as a base. The program  ========
#====== isolates large objects of specific color and    ========
#====== supplies information - horizontal and vertical  ========
#====== position respectful of the frame center as a    ========
#====== list of string objects through the serial port  ========
#====== to the Arduino. The third object in the list is ========
#====== the approximation of the distance dependant on  ========
#====== the area of the object that is been tracked as  ========
#======             it appears in the camera            ========
#===============================================================

import cv2
import numpy as np
import serial
import sys

serial_port = serial.Serial('/dev/ttyACM0', 9600) # Ports might be different depending on the setup being used

center_frame = (320,240)    # The (x,y) coordinates of the center of the frame with the resolution 640*480
radius_frame = (140)        # The minimum desired radius of the object being tracked
area_frame = 61575          # The desired area of the object that is being tracked
radius_frame_max = (200)    # The maximum desired radius of the object being tracked
area_frame_max = 125663     # The maximum desired area of the object being tracked

source = cv2.VideoCapture(0)
while(1):
    kernel_open = np.ones((10,10),np.uint8)     # Erosion values
    kernel_close = np.ones((30,30),np.uint8)    # Dilution values
    _, frame = source.read()                    # reads one frame at a time

    # Use this to get the resolution of the picture
    # print(frame.shape)

    # BGR to HSV conversion helps better isolate a single color
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_blue = np.array([50,50,50])   # define range of green color in HSV
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

    # Use this to see the array of the object
    # print(contours)

    # Two circle objects being drawn as a visual representation
    # of what the minimum and maximum desired areas are for the object being tracked
    cv2.circle(res,center_frame,radius_frame,(255,0,0),2)
    cv2.circle(res,center_frame,radius_frame_max,(0,0,255),2)

    # the following algorithm calculates the countours and the parameters needed for tracking
    # in case an object of green color and sufficient size is identified
    if len(contours) > 0:
        cnt = contours[0]
        (x,y),radius_obj = cv2.minEnclosingCircle(cnt)      # Parameters (center, radius) of the minimum circle that can enclose the tracked object
        center_obj = (int(x),int(y))                        # Center of the tracked object
        radius_obj = int(radius_obj)                        # Radius of the tracked object
        cv2.circle(res,center_obj,radius_obj,(0,255,0),-1)  # Visual representation of the minimum circular enclosure

        area_obj = ((radius_obj**2)*3.14159265359)          # Area of the minimum circular enclosure

        # Distance comparison between the calculated area and the desired area
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
        # The list of string objects that is sent to the Arduino.
        # The objects found are:
        # 1. Identification - used by the arduino to identify the first element
        # 2. Position in the Horizontal Plane with respect to the center of the frame
        # 3. Position in the Vertical Plane with respect to the center of the frame
        # 4. Distance approximation can only be judged as one of the three:
        #       a. 0 - too close to the camera at which the drone should back up
        #       b. 1 - ideal position at which the drone should remain hovering
        #       c. 2 - too far away - the drone should keep moving towards the object
        to_be_sent = ["31415", offset_hor, offset_ver, distance]

        for i in range(len(to_be_sent)):
            serial_port.write(to_be_sent[i])

    cv2.imshow('res',res)
    cv2.imshow('frame',frame)
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()
