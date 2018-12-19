#!/usr/bin/python

import matplotlib.pyplot as plt
from pybathy_maps import patch_draper, map_draper, mesh_map
from pydata_tools import data_structures, gsf_data, utils
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

def plot_patch_views(patch_views):
    world_size = 8.

    max_patches = max(len(p.sss_views) for p in patch_views)

    f, axarr = plt.subplots(len(patch_views), max_patches+1)
    for i, p in enumerate(patch_views):

        axarr[i, 0].imshow(p.patch_height, cmap=plt.jet())
        print "Patch view dirs: ", p.patch_view_dirs

        for j, (im, pos, d) in enumerate(zip(p.sss_views, p.patch_view_pos, p.patch_view_dirs)):
            axarr[i, j+1].imshow(im, vmin=0., vmax=1., cmap=plt.gray())

            image_size = float(im.shape[0])
            impos = image_size/world_size*pos
            x, y = impos[0], impos[1]
            if abs(x) < .5*image_size and abs(y) < .5*image_size:
                pass
            elif abs(y) < abs(x):
                y = .5*image_size*y/abs(x)
                x = .5*image_size*sign(x)
            else:
                x = .5*image_size*x/abs(y)
                y = .5*image_size*sign(y)

            x = x + .5*image_size
            y = y + .5*image_size

            print impos
            print x, y
            print d

            axarr[i, j+1].set_title("Dist: %d" % int(np.linalg.norm(pos[:2])))
            axarr[i, j+1].scatter(x, y, s=20, c='white', marker='o')
            axarr[i, j+1].arrow(x, y, 10.*d[0], 10.*d[1], color='red', width=0.01, head_width=0.1)

        for j in range(len(p.sss_views), max_patches):
            axarr[i, j+1].axis('off')

    plt.show()

map_images = map_draper.sss_map_image.read_data("map_images_cache.cereal")

map_image = map_images[0]
resolution = (map_image.bounds[1, 0]-map_image.bounds[0, 0])/float(map_image.sss_map_image.shape[1])

height_map = generate_or_load_height_map(sys.argv[1], resolution)

print "Height map shape: ", height_map.shape
print "SSS map shape: ", map_image.sss_map_image.shape

patch_views = map_draper.convert_maps_to_patches(map_images, height_map, 8.)

print "Number of patches :", len(patch_views)

for i in range(0, len(patch_views), 10):
    plot_patch_views(patch_views[i:i+10])
