#!/usr/bin/python

import cv2
import numpy as np

img = cv2.imread("../data/large_multibeam_survey.png")
height, width, channels = img.shape
patch_size = 300
patch_step = patch_size/2
patches_height = 5
patches_width = 10

print "Channels: ", channels

real_width = 30000. # meters
real_height = float(height)/float(width)*real_width

x = np.linspace(0, real_height, height)
y = np.linspace(0, real_width, width)
xv, yv = np.meshgrid(y, x)

z = img[:, :, 0].reshape(height, width).astype(float)

z = 500./255.*z

print "Xv: ", xv.shape

print "Yv: ", yv.shape

points = np.stack((xv, yv, z), axis=2)
points_list = points.reshape(height*width, 3)

for ii in range(0, patches_height):
    for jj in range(0, patches_width):
        istart = patch_step*ii
        iend = istart + patch_size
        jstart = patch_step*jj
        jend = jstart + patch_size
        pointsij = points[istart:iend, jstart:jend].reshape(-1, 3)
        filename = "patch_%02d_%02d.xyz" % (ii, jj)
        with open(filename, 'a') as xyz_file:
            for i, p in enumerate(pointsij):
                #if i % 100 != 0:
                #    continue
                xyz_file.write('%f %f %f\n'%(p[0], p[1], p[2]))

#with open('points.xyz', 'a') as xyz_file:
#    for i, p in enumerate(points):
#        if i % 100 != 0:
#            continue
#        xyz_file.write('%f %f %f\n'%(p[0], p[1], p[2]))


