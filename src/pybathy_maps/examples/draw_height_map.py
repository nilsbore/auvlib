#!/usr/bin/python

from auvlib.data_tools import std_data, gsf_data, utils
from auvlib.bathy_maps import draw_map
import sys
import os

def parse_or_load_gsf(path):

    if os.path.exists("gsf_cache.cereal"):
        gsf_pings = gsf_data.gsf_mbes_ping.read_data("gsf_cache.cereal")
    else:
        gsf_pings = gsf_data.gsf_mbes_ping.parse_folder(path)
        gsf_data.write_data(gsf_pings, "gsf_cache.cereal")
    return gsf_pings

gsf_pings = utils.parse_or_load_gsf(sys.argv[1])
mbes_pings = gsf_data.convert_pings(gsf_pings)

d = draw_map.BathyMapImage(mbes_pings, 500, 500)
d.draw_height_map(mbes_pings)
d.draw_track(mbes_pings)
d.write_image("height_map.png")
d.show()
