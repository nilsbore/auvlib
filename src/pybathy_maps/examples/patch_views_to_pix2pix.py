#!/usr/bin/python

import matplotlib.pyplot as plt
from auvlib.bathy_maps import patch_draper, data_vis
from numpy import sign
import numpy as np
from copy import deepcopy
import sys
import os
import cv2
from auvlib.bathy_maps.gen_utils import clip_to_interval

def filter_sss_coverage(patch_views):
    for view in patch_views:
        view.sss_views = filter(lambda s: np.mean(s == 0) < 0.2, view.sss_views)
    patch_views = filter(lambda s: len(s.sss_views) > 0, patch_views)
    return patch_views

def filter_map_coverage(patch_views):
    patch_views = filter(lambda s: np.all(s.patch_height > 0) or np.mean(s.patch_height == 0) < 0.1, patch_views)
    return patch_views

def filter_map_nans(patch_views):
    patch_views = filter(lambda s: not np.any(np.isnan(s.patch_height)), patch_views)
    patch_views = filter(lambda s: not np.any(np.isinf(s.patch_height)), patch_views)
    return patch_views

def filter_map_extremas(patch_views, interval):
    patch_views = filter(lambda s: np.mean(s.patch_height == 0) < 0.1, patch_views)
    patch_views = filter(lambda s: np.mean(s.patch_height == interval) < 0.1, patch_views)
    return patch_views

patch_views = patch_draper.sss_patch_views.read_data(sys.argv[1])
#patch_views.append(patch_views[0])

print "Number of patches before sss filtering: ", len(patch_views)

patch_views = filter_sss_coverage(patch_views)

print "Number of patches after sss filtering: ", len(patch_views)

patch_views = filter_map_coverage(patch_views)

print "Number of patches after map filtering: ", len(patch_views)

interval = 2.
for view in patch_views:
    view.patch_height = clip_to_interval(view.patch_height, interval)

patch_views = filter_map_nans(patch_views)

print "Number of patches after map nan filtering: ", len(patch_views)

patch_views = filter_map_extremas(patch_views, interval)

print "Number of patches after map extremas filtering: ", len(patch_views)

#for i in range(0, len(patch_views), 10):
#    data_vis.plot_patch_views(patch_views[i:i+10])

if not os.path.exists("pix2pix_sonar"):
    os.makedirs("pix2pix_sonar")
    os.makedirs(os.path.join("pix2pix_sonar", "8"))
    os.makedirs(os.path.join("pix2pix_sonar", "16"))
    os.makedirs(os.path.join("pix2pix_sonar", "24"))
    os.makedirs(os.path.join("pix2pix_sonar", "32"))

height, width = 256, 256
for i, view in enumerate(patch_views):
    sss_patch = cv2.resize(view.sss_views[0], (width, height), interpolation = cv2.INTER_CUBIC)
    height_patch = cv2.resize(view.patch_height, (width, height), interpolation = cv2.INTER_CUBIC)
    height_patch = (255/interval*height_patch).astype(np.uint8)

    height_image = cv2.applyColorMap(height_patch, cv2.COLORMAP_JET)
    sss_image_gray = (255.*np.minimum(np.maximum(sss_patch, 0.), 1.)).astype(np.uint8)
    sss_image = cv2.cvtColor(sss_image_gray, cv2.COLOR_GRAY2RGB)

    concatenated = np.concatenate((sss_image, height_image), axis=1)

    dist = np.linalg.norm(view.patch_view_pos[:2])
    if abs(dist - 8.) < 1.:
        filename = os.path.join("pix2pix_sonar", "8", "%d.jpg" % i)
    elif abs(dist - 16.) < 1.:
        filename = os.path.join("pix2pix_sonar", "16", "%d.jpg" % i)
    elif abs(dist - 24.) < 1.:
        filename = os.path.join("pix2pix_sonar", "24", "%d.jpg" % i)
    elif abs(dist - 32.) < 1.:
        filename = os.path.join("pix2pix_sonar", "32", "%d.jpg" % i)
    else:
        continue
    cv2.imwrite(filename, concatenated)
