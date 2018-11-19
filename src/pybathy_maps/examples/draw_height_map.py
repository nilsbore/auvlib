#!/usr/bin/python

from pydata_tools import data_structures, gsf_data
from pybathy_maps import draw_map
import sys
import os

def parse_or_load_gsf(path):

    if os.path.exists("gsf_cache.cereal"):
        gsf_pings = gsf_data.gsf_mbes_ping.read_data("gsf_cache.cereal")
    else:
        gsf_pings = gsf_data.gsf_mbes_ping.parse_folder(path)
        gsf_data.write_data(gsf_pings, "gsf_cache.cereal")
    return gsf_pings

gsf_pings = parse_or_load_gsf(sys.argv[1])
mbes_pings = gsf_data.convert_pings(gsf_pings)

d = draw_map.bathy_map_image(mbes_pings, 500, 500)
d.draw_height_map(mbes_pings)
d.draw_track(mbes_pings)
d.save_image("height_map.png")
