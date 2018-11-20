#!/usr/bin/python

from pydata_tools import data_structures, all_data
from pybathy_maps import draw_map, mesh_map
import sys
import os

all_pings = all_data.all_mbes_ping.parse_folder(sys.argv[1])
all_entries = all_data.all_nav_entry.parse_folder(sys.argv[1])

mbes_pings = all_data.convert_matched_entries(all_pings, all_entries)

d = draw_map.bathy_map_image(mbes_pings, 500, 500)
d.draw_height_map(mbes_pings)
d.draw_track(mbes_pings)
d.save_image("height_map.png")

m = mesh_map.bathy_map_mesh()
V, F, bounds = m.mesh_from_pings(mbes_pings, 2.0)
m.display_mesh(V, F)
