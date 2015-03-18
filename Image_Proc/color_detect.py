import cv2
import numpy as np
import serial
import sys

ser = serial.Serial('/dev/ttyACM0', 9600)

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
    lower_blue = np.array([105,50,50])
    upper_blue = np.array([135,255,255])

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
    # additional methods can be used on contours in order to find the area/perimeter and the center

    if len(contours) > 0:
        cnt = contours[0]
        (x,y),radius = cv2.minEnclosingCircle(cnt)
        center = (int(x),int(y))
        radius = int(radius)
        cv2.circle(res,center,radius,(0,255,0),-1)
        send_val = (str(int(center[0])) + "\n")
        ser.write(send_val)
        # Print center in order to now the center of the image print (center)
    else:
        ser.write('0')

    cv2.imshow('res',res)
    cv2.imshow('frame',frame)
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()
