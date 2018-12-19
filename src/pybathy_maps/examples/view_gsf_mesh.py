#!/usr/bin/python

from pydata_tools import data_structures, gsf_data, xtf_data, csv_data, utils
from pybathy_maps import mesh_map, patch_draper
import sys
import os
import numpy as np
from numpy import sign
import math
import matplotlib.pyplot as plt

def create_mesh(path):

    gsf_pings = utils.parse_or_load_gsf(path)
    mbes_pings = gsf_data.convert_pings(gsf_pings)
    V, F, bounds = mesh_map.mesh_from_pings(mbes_pings, 0.5)

    return V, F, bounds

def match_or_load_xtf(xtf_path, csv_path):

    if os.path.exists("matched_cache.cereal"):
        xtf_pings = xtf_data.xtf_sss_ping.read_data("matched_cache.cereal")
    else:
        xtf_pings = utils.parse_or_load_xtf(xtf_path)
        nav_entries = utils.parse_or_load_csv(csv_path)
        xtf_pings = csv_data.convert_matched_entries(xtf_pings, nav_entries)
        xtf_data.write_data(xtf_pings, "matched_cache.cereal")
    return xtf_pings

def plot_patch_views(patch_views):
    world_size = 8.

    max_patches = max(len(p.sss_views) for p in patch_views)

    f, axarr = plt.subplots(max(len(patch_views), 2), max_patches+1)
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

class PatchPlotter(object):

    def __init__(self):

        self.patches = []

    def plot_callback(self, patch_views):

        print "Got patch callback!"
        self.patches.append(patch_views)
        patch_draper.write_data(self.patches, "patch_views_cache.cereal")
        plot_patch_views(self.patches)

sensor_yaw = 5.*math.pi/180.
sensor_offset = np.array([2., -1.5, 0.])

V, F, bounds = create_mesh(sys.argv[1])
#mesh_map.show_mesh(V, F)

xtf_pings = match_or_load_xtf(sys.argv[2], sys.argv[3])
xtf_pings = xtf_data.correct_sensor_offset(xtf_pings, sensor_offset)
sound_speeds = csv_data.csv_asvp_sound_speed.parse_file(sys.argv[4])

plotter = PatchPlotter()
#patch_draper.drape_viewer(V, F, bounds, xtf_pings, sound_speeds, sensor_yaw)
#patch_views = patch_draper.drape_patches(V, F, bounds, xtf_pings, sound_speeds, sensor_yaw, plotter.plot_callback)

Vb, Fb, Cb = patch_draper.get_vehicle_mesh()
viewer = patch_draper.BaseDraper(V, F, xtf_pings, bounds, sound_speeds)
#viewer = patch_draper.PatchDraper(V, F, xtf_pings, bounds, sound_speeds)
viewer.set_sidescan_yaw(sensor_yaw)
viewer.set_vehicle_mesh(Vb, Fb, Cb)
viewer.set_ray_tracing_enabled(False)
#viewer.set_patch_callback(plotter.plot_callback)
viewer.show()
#patch_views = viewer.get_patch_views()
#patch_draper.write_data(patch_views, "patch_views.cereal")
