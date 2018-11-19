#!/usr/bin/python

from pydata_tools import data_structures, gsf_data, xtf_data, csv_data
from pybathy_maps import mesh_map
import sys
import os

def parse_or_load_gsf(path):

    if os.path.exists("gsf_cache.cereal"):
        gsf_pings = gsf_data.read_data("gsf_cache.cereal")
    else:
        gsf_pings = gsf_data.parse_folder(path)
        gsf_data.write_data(gsf_pings, "gsf_cache.cereal")
    return gsf_pings

def parse_or_load_xtf(path):

    if os.path.exists("xtf_cache.cereal"):
        xtf_pings = xtf_data.read_data("xtf_cache.cereal")
    else:
        xtf_pings = xtf_data.parse_folder(path)
        xtf_data.write_data(xtf_pings, "xtf_cache.cereal")
    return xtf_pings

def create_mesh(path):

    gsf_pings = parse_or_load_gsf(path)
    mbes_pings = gsf_data.convert_pings(gsf_pings)
    m = mesh_map.bathy_map_mesh()
    V, F, bounds = m.mesh_from_pings(mbes_pings, 0.5)

    return m, V, F, bounds

m, V, F, bounds = create_mesh(sys.argv[1])
#m.display_mesh(V, F)

xtf_pings = parse_or_load_xtf(sys.argv[2])
nav_entries = csv_data.parse_file(sys.argv[3])
xtf_pings = csv_data.convert_matched_entries(xtf_pings, nav_entries)

m.overlay_sss(V, F, bounds, xtf_pings)
