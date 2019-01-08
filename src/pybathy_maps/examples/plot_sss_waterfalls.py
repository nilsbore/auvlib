#!/usr/bin/python

import matplotlib.pyplot as plt
from auvlib.bathy_maps import patch_draper, draping_image, mesh_map
from auvlib.data_tools import std_data, gsf_data
from numpy import sign
import numpy as np
from copy import deepcopy
import sys
import os

def plot_waterfalls(map_image):
    
    f, axarr = plt.subplots(1, 2)

    axarr[0].imshow(map_image.sss_waterfall_image, cmap=plt.gray(), interpolation='none')
    axarr[1].imshow(map_image.sss_waterfall_depth, cmap=plt.jet(), interpolation='none')

    plt.show()

map_images = draping_image.sss_map_image.read_data("map_images_cache.cereal")

for map_image in map_images:
    plot_waterfalls(map_image)
