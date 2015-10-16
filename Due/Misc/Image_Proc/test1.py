import cv2
import numpy as np

cap = cv2.VideoCapture(0)

while(1):
    kernel_open = np.ones((20,20),np.uint8)
    kernel_close = np.ones((21,21),np.uint8)
    # Take each frame
    _, frame = cap.read()

    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    imgray = cv2.cvtColor(hsv,cv2.COLOR_BGR2GRAY)
    opening = cv2.morphologyEx(imgray, cv2.MORPH_OPEN, kernel_open)
    closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel_close)


    ret,thresh = cv2.threshold(closing,127,255,0)
    contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(closing, contours, 0, (0,255,0), 3)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame,frame, mask= mask)

    cv2.imshow('frame',closing)
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()
