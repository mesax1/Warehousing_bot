import numpy as np
import argparse
import imutils
import cv2

ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required = True, help = "Path to the image")
args = vars(ap.parse_args())

image = cv2.imread(args["image"])
cv2.imshow("Original", image)

resized = imutils.resizespec(image, width= 130, height = 145)
cv2.imshow("Resized via Function", resized)
print(args)
cv2.imwrite("newImage.pgm", resized)

cv2.waitKey(0)
