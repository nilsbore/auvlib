#!/usr/bin/python

#import pylab as pl
from matplotlib import pyplot as plt
import numpy as np
import cv2
import sys

name = sys.argv[1]
minv = float(sys.argv[2])
maxv = float(sys.argv[3])
path = name + "_colorbar_jet.png"

print "Got minv: ", minv, " maxv: ", maxv
print "Saving to image file: ", path

plt.figure(figsize=(3.5,3.5))
img = plt.imshow(np.array([[minv,maxv]]), cmap="jet")
img.set_visible(False)
img.axes.set_visible(False)

plt.colorbar(orientation="vertical", label="Consistency Error (m)")
plt.savefig(path)

image = cv2.imread(path)
cropped = image[30:-30, -image.shape[1]/2+90:]
cv2.imwrite(path, cropped)
