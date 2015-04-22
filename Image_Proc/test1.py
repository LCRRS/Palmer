import cv2
import cv2.cv as cv
cv.NamedWindow ('CamShiftDemo', 1)
device = -1
cap = cv.CaptureFromCAM(device)
size = (320,240)
cv.SetCaptureProperty(cap, cv.CV_CAP_PROP_FPS,10)
cv.SetCaptureProperty(cap, cv.CV_CAP_PROP_FRAME_WIDTH, size[0])
cv.SetCaptureProperty(cap, cv.CV_CAP_PROP_FRAME_HEIGHT, size[1])
while True:
    frame = cv.QueryFrame(cap)
    cv.ShowImage('CamShiftDemo', frame)
    cv.WaitKey(10)
