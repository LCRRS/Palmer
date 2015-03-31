#===============================================================
#===============         COLOR TRACKING          ===============
#===============================================================

import cv2
import cv2.cv as cv
import numpy as np
import serial
import sys
import math as mt

serial_port = serial.Serial('/dev/ttyACM0', 115200) # Ports might be different depending on the setup being used

#=============================================
#=============== PID DEFINITIONS ================
#=============================================

center_frame = (88,72)    # The (x,y) coordinates of the center of the frame with the resolution 640*480
radius_frame = (30)        # The minimum desired radius of the object being tracked
area_frame = 2827          # The desired area of the object that is being tracked
radius_frame_max = (50)    # The maximum desired radius of the object being tracked
area_frame_max = 7854   # The maximum desired area of the object being tracked
size = (240, 180)         # The resolution of the camera

Kp_dist = 0.125                # Proportionality constant for the PID calculation of the Distance
Kd_dist = 0.03                # Derivative constant for the PID calculation of Distance
Upper_Limit_dist = 3.0       # Upper limit for the PID output of the distance correction
Lower_Limit_dist = -3.0      # Lower limit for the PID output of the distance correction
Setpoint_dist = 100          # The desired position of the quadcopter in centimeters
PID_output_dist = 0
first_calculation = None

Kp_ver = 0.125

#=============================================
#=============== PID FUNCTION ================
#=============================================

def PID_Compute(Kp, Kd, Upper_Limit, Lower_Limit, Set_Point, Pid_Input, Pid_Output):
    global first_calculation
    global last_input
    if first_calculation == None:
        last_input = Pid_Input
        first_calculation = 0
        print ("last input is :     ", last_input)

    # Setting the variables
    error = Set_Point - Pid_Input
    d_input = Pid_Input - last_input

    #Computing the Output
    output = (error * Kp) + (d_input * Kd)
    print("Last input is :     ", last_input)
    # Remember the last error for the next Kd calculation
    last_input = Pid_Input
    print ("Last input is :    ", last_input)

    if output > Upper_Limit:
        output = Upper_Limit
        return output
    if output < Lower_Limit:
        output = Lower_Limit
        return output
    else:
        return output



source = cv2.VideoCapture(0)

# Set the camera resolution (if supported by your particular camera)
ret = source.set(cv.CV_CAP_PROP_FRAME_WIDTH,size[0])
ret = source.set(cv.CV_CAP_PROP_FRAME_HEIGHT,size[1])
# VideoCapture::get(CV_CAP_PROP_POS_FRAMES).
# source.set(5,1)

while(1):
    kernel_open = np.ones((5,5),np.uint8,5)     # Erosion values
    kernel_close = np.ones((5,5),np.uint8,5)    # Dilution values
    _, frame = source.read()                    # reads one frame at a time

    # Use this to get the resolution of the picture
    # print(frame.shape)

    # BGR to HSV conversion helps better isolate a single color
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Definition of Fifty Shades of Green
    lower_blue = np.array([30,50,50])   # define range of green color in HSV
    upper_blue = np.array([80,255,255])

    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)
    closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel_close)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame,frame, mask= closing)
    imgray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
    ret,thresh = cv2.threshold(imgray,1,255,0)
    contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    # Use to see the array of the object
    # print(contours)

    # the following algorithm calculates the countours and the parameters needed for tracking
    # in case an object of green color and sufficient size is identified
    if len(contours) > 0:
        cnt = contours[0]
        (x,y),radius_obj = cv2.minEnclosingCircle(cnt)      # Parameters (center, radius) of the minimum circle that can enclose the tracked object
        center_obj = (int(x),int(y))                        # Center of the tracked object
        radius_obj = int(radius_obj)                        # Radius of the tracked object

        area_obj = ((radius_obj**2)*3.14159265359)          # Area of the minimum circular enclosure

        # Distance comparison between the calculated area and the desired area

        distance = 6.573/(mt.tan(((radius_obj/3.52)*mt.pi)/180))
        pid_distance = PID_Compute(Kp_dist, Kd_dist, Upper_Limit_dist, Lower_Limit_dist, Setpoint_dist, distance, PID_output_dist)
        offset_dis = str(int(pid_distance*100))+"\n"

        # Produces the new PID_yaw setpoint "-" corresponds to the CCW activation; "+" to the CW activation
        # Value "3.52" is the number of degrees per pixel in the current camera resolution
        offset_hor = (str(int((((center_obj[0]) - center_frame[0])/3.52)*10))+"\n")
        # Alters the START_SPEED where "-" corresponds to the decrease in thrust and "+" to the increase in thrust
        offset_ver = (str(int((((center_frame[1]) - center_obj[1])*Kp_ver)*10))+"\n")
        # The list of string objects that is sent to the Arduino.
        # The objects found are:
        # 1. Identification - used by the arduino to identify the first element
        # 2. Position in the Horizontal Plane with respect to the center of the frame
        # 3. Position in the Vertical Plane with respect to the center of the frame
        # 4. Distance approximation can only be judged as one of the three:
        #       a. 0 - too close to the camera at which the drone should back up
        #       b. 1 - ideal position at which the drone should remain hovering
        #       c. 2 - too far away - the drone should keep moving towards the object
        to_be_sent = ["314159.0\n", offset_hor, offset_ver, offset_dis]
        for i in range(len(to_be_sent)):
            serial_port.write(to_be_sent[i])
            print(to_be_sent[i])

    else:
        to_be_sent = ["314159.0\n", "0.0\n", "0.0\n", "0.0\n"]
        for i in range(len(to_be_sent)):
            serial_port.write(to_be_sent[i])

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()
