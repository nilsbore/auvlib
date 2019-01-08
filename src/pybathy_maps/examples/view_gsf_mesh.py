#!/usr/bin/python

from auvlib.data_tools import std_data, gsf_data, xtf_data, csv_data, utils
from auvlib.bathy_maps import mesh_map, patch_draper, data_vis
import sys
import os
import numpy as np
import math

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


class PatchPlotter(object):

    def __init__(self):

        self.patches = []

    def plot_callback(self, patch_views):

        print "Got patch callback!"
        self.patches.append(patch_views)
        patch_draper.write_data(self.patches, "patch_views_cache.cereal")
        data_vis.plot_patch_views(self.patches)

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
#viewer = patch_draper.BaseDraper(V, F, xtf_pings, bounds, sound_speeds)
viewer = patch_draper.PatchDraper(V, F, xtf_pings, bounds, sound_speeds)
viewer.set_sidescan_yaw(sensor_yaw)
viewer.set_vehicle_mesh(Vb, Fb, Cb)
viewer.set_ray_tracing_enabled(False)
viewer.set_patch_callback(plotter.plot_callback)
viewer.show()
patch_views = viewer.get_patch_views()
patch_draper.write_data(patch_views, "patch_views.cereal")
