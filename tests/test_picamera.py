from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2

#Capture image as array
camera = PiCamera()
rawImage = PiRGBArray(camera)
time.sleep(0.1)
camera.capture(rawImage, format = "rgb")
image = rawImage.array

# Record captured image on MicroSD card
cv2.imwrite("test-picam.jpeg", image)