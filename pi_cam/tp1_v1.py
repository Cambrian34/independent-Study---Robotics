#this was designed to be run on a raspberry pi with a camera module
#this code will take a picture and show it using cv2

import cv2
import time
import picamera2 as picamera

#initialize the camera
camera = picamera.PiCamera()

#take a picture
camera.capture('test.jpg')

#read the picture
img = cv2.imread('test.jpg')

#show the picture
cv2.imshow('image', img)
cv2.waitKey(0)
cv2.destroyAllWindows()

#clean up
camera.close()


