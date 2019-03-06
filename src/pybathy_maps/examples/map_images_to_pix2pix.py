#!/usr/bin/python

import matplotlib.pyplot as plt
from auvlib.bathy_maps import patch_draper, map_draper, mesh_map, data_vis, gen_utils
from auvlib.data_tools import std_data, gsf_data, utils
from numpy import sign
import numpy as np
from copy import deepcopy
import sys
import os
import cv2

def convert_wf_to_slices(wf_sss, wf_depth, image_height=32):

    image_width = wf_sss.shape[1]/2
    wf_length = wf_sss.shape[0]

    sss_slices = []
    depth_slices = []

    for i in xrange(0, wf_length-image_height, image_height/2):
        sss_slice_left = wf_sss[i:i+image_height, :image_width]
        depth_slice_left = wf_depth[i:i+image_height, :image_width]
        sss_slice_right = wf_sss[i:i+image_height, image_width:]
        depth_slice_right = wf_depth[i:i+image_height, image_width:]

        sss_slices.append(sss_slice_right)
        depth_slices.append(depth_slice_right)
        sss_slices.append(np.fliplr(sss_slice_left))
        depth_slices.append(np.fliplr(depth_slice_left))

    return sss_slices, depth_slices

def normalize_depth_slices(depth_slices):

    depth_slices = [gen_utils.clip_to_interval(d, 2.)/2. for d in depth_slices]
    for d in depth_slices:
        d[d==1.] = 0.

    return depth_slices

def visualize_slices(sss_slices, depth_slices):

    for s, d in zip(sss_slices, depth_slices):
        cv2.imshow("SSS Slice", s)
        cv2.imshow("Depth slice", d)
        cv2.waitKey()

def rescale_slices(slices):

    #slices = [(255.*cv2.resize(np.rot90(s), (256, 256), interpolation=cv2.INTER_CUBIC)).astype(np.uint8) for s in slices]

    slices = [cv2.resize(np.rot90(s), (256, 256), interpolation=cv2.INTER_NEAREST) for s in slices]
    slices = [(255.*np.minimum(np.maximum(s, 0.), 1.)).astype(np.uint8) for s in slices]

    return slices

def erode_depth_slices(depth_slices):

    for d in depth_slices:
        b = (d == 0.).astype(np.uint8)
        kernel = np.ones((3, 3), np.uint8)
        b = cv2.dilate(b, kernel, iterations=1).astype(bool)
        d[b] = 0.

    return depth_slices

def normalize_intensities(sss_slices):

    new_slices = []
    for s in sss_slices:
        m = np.mean(s[s>.1])
        ss = s.copy()
        if m != 0 and not np.isnan(m):
            ss *= .3/m
        new_slices.append(ss)

    return new_slices

def scale_slices(slices, image_height, image_width):

    #slices = [(255.*cv2.resize(np.rot90(s), (256, 256), interpolation=cv2.INTER_CUBIC)).astype(np.uint8) for s in slices]
    slices = [cv2.resize(np.rot90(s), (image_width, image_height), interpolation=cv2.INTER_LINEAR) for s in slices]

    return slices


map_images = map_draper.sss_map_image.read_data("map_images_cache.cereal")
image_height = 64
image_width = map_images[0].sss_waterfall_image.shape[1]/2
dataset_name = "pix2pix_cos2"

sss_slices = []
depth_slices = []
for map_image in map_images:
    #s, d = convert_wf_to_slices(map_image.sss_waterfall_image, map_image.sss_waterfall_depth, image_height)
    s, d = convert_wf_to_slices(map_image.sss_waterfall_image, map_image.sss_waterfall_model, image_height)
    sss_slices.extend(s)
    depth_slices.extend(d)

#depth_slices = normalize_depth_slices(depth_slices)
sss_slices = normalize_intensities(sss_slices)
#depth_slices = erode_depth_slices(depth_slices)
depth_slices = rescale_slices(depth_slices)
sss_slices = rescale_slices(sss_slices)

#sss_slices = scale_slices(sss_slices, image_height, image_width)

#visualize_slices(sss_slices, depth_slices)


if not os.path.exists(dataset_name):
    os.makedirs(dataset_name)

for i, (s, d) in enumerate(zip(sss_slices, depth_slices)):
    concatenated = np.concatenate((s, d), axis=1)
    filename = os.path.join(dataset_name, "%d.jpg" % i)
    cv2.imwrite(filename, concatenated)
