#!/usr/bin/python

from auvlib.data_tools import std_data, all_data
from auvlib.bathy_maps import draw_map, mesh_map
import sys
import os

all_pings = all_data.all_mbes_ping.parse_folder(sys.argv[1])
all_entries = all_data.all_nav_entry.parse_folder(sys.argv[1])

mbes_pings = all_data.convert_matched_entries(all_pings, all_entries)

d = draw_map.BathyMapImage(mbes_pings, 500, 500)
d.draw_height_map(mbes_pings)
d.draw_track(mbes_pings)
d.write_image("all_height_map.png")
d.show()

V, F, bounds = mesh_map.mesh_from_pings(mbes_pings, 2.0)
mesh_map.show_mesh(V, F)
