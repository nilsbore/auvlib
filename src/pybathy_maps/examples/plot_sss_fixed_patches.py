#!/usr/bin/python

import matplotlib.pyplot as plt
from auvlib.bathy_maps import patch_draper, map_draper, mesh_map, data_vis
from auvlib.data_tools import std_data, gsf_data, utils
from numpy import sign
import numpy as np
from copy import deepcopy
import sys
import os

def generate_or_load_height_map(path, resolution):

    if os.path.exists("height_map.npz"):
        #height_map = gsf_data.gsf_mbes_ping.read_data("gsf_cache.cereal")
        height_map = np.load("height_map.npz")["height_map"]
    else:
        gsf_pings = utils.parse_or_load_gsf(path)
        mbes_pings = gsf_data.convert_pings(gsf_pings)
        m = mesh_map.bathy_map_mesh()
        height_map, bounds = m.height_map_from_pings(mbes_pings, resolution)
        #gsf_data.write_data(gsf_pings, "gsf_cache.cereal")
        np.savez("height_map.npz", height_map=height_map)
    return height_map

map_images = map_draper.sss_map_image.read_data("map_images_cache.cereal")

map_image = map_images[0]
resolution = (map_image.bounds[1, 0]-map_image.bounds[0, 0])/float(map_image.sss_map_image.shape[1])

height_map = generate_or_load_height_map(sys.argv[1], resolution)

print "Height map shape: ", height_map.shape
print "SSS map shape: ", map_image.sss_map_image.shape

patch_views = map_draper.convert_maps_to_single_angle_patches(map_images, height_map, 8.)

print "Number of patches :", len(patch_views)

patch_draper.write_data(patch_views, "fixed_patch_views.cereal")

#for i in range(0, len(patch_views), 10):
#    data_vis.plot_patch_views(patch_views[i:i+10])

