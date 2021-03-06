"""
Image interpolation
=====================

The example demonstrates image interpolation on a Racoon face.
"""

import scipy.misc
import matplotlib.pyplot as plt

#f = scipy.misc.face(gray=True)
f= scipy.misc.imread('octomapwarehouse9.pgm')
g= scipy.misc.imread('octomapwarehouse8.pgm')

plt.figure(figsize=(8, 4))

plt.subplot(1, 2, 1)
plt.imshow(f, cmap=plt.cm.gray)
plt.axis('off')

plt.subplot(1, 2, 2)
plt.imshow(f, cmap=plt.cm.gray, interpolation='lanczos')
plt.axis('off')

plt.subplots_adjust(wspace=0.02, hspace=0.02, top=1, bottom=0, left=0, right=1)
plt.show()
