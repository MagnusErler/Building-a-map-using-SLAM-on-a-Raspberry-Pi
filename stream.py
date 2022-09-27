import cv2
import numpy as np
cap1 = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L)
while True:
    ret1,img1 = cap1.read()
    cv2.imshow('video output1', img1)
    k=cv2.waitKey(10)& 0xff
    if k==27:
        break
cap1. release()
cv2.destroyAllWindows()