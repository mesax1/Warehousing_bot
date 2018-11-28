import numpy as np
import argparse
import cv2

ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required = True, help = "Path to the image")
ap.add_argument("-i2", "--image2", required = True, help = "Path to the image")
args = vars(ap.parse_args())
print(args)

image1 = cv2.imread(args["image"])
cv2.imshow("Map 1", image1)

image2 = cv2.imread(args["image2"])
cv2.imshow("Map 2", image2)


bitwiseNot1 = cv2.bitwise_not(image1)
#cv2.imshow("NOT", bitwiseNot1)

bitwiseNot2 = cv2.bitwise_not(image2)
#cv2.imshow("NOT", bitwiseNot2)

#bitwiseAnd = cv2.bitwise_and(image1, image2)
bitwiseAnd = cv2.bitwise_and(bitwiseNot1, bitwiseNot2)
cv2.imshow("AND", bitwiseAnd)

bitwiseNot= cv2.bitwise_not(bitwiseAnd)
cv2.imshow("NOT", bitwiseNot)

cv2.imwrite("ANDmap.pgm", bitwiseNot)


cv2.waitKey(0)


#python andimages.py --image ~/OpenCVBook/images/resmap1.pgm --image2 ~/OpenCVBook/images/resmap3.pgm
