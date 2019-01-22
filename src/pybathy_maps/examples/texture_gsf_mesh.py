#!/usr/bin/python

from auvlib.data_tools import std_data, gsf_data, utils
from auvlib.bathy_maps import mesh_map
import sys
import numpy as np

gsf_pings = utils.parse_or_load_gsf(sys.argv[1])
mbes_pings = gsf_data.convert_pings(gsf_pings)
V, F, bounds = mesh_map.mesh_from_pings(mbes_pings, 0.5)

height_map, bounds = mesh_map.height_map_from_pings(mbes_pings, 0.2)
#mesh_map.show_height_map(height_map)

R, G, B = mesh_map.height_map_to_texture(height_map)

mesh_map.show_textured_mesh(V, F, R, G, B, bounds)

#mesh_map.show_mesh(V, F)

