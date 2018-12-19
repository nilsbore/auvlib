#!/usr/bin/python

from pydata_tools import data_structures, gsf_data, xtf_data, csv_data
from pybathy_maps import draw_map, mesh_map, patch_draper
import sys
import os

def parse_or_load_gsf(path):

    if os.path.exists("gsf_cache.cereal"):
        gsf_pings = gsf_data.gsf_mbes_ping.read_data("gsf_cache.cereal")
    else:
        gsf_pings = gsf_data.gsf_mbes_ping.parse_folder(path)
        gsf_data.write_data(gsf_pings, "gsf_cache.cereal")
    return gsf_pings

def parse_or_load_xtf(xtf_path, csv_path):

    if os.path.exists("xtf_cache.cereal"):
        xtf_pings = xtf_data.xtf_sss_ping.read_data("xtf_cache.cereal")
    else:
        xtf_pings = xtf_data.xtf_sss_ping.parse_folder(xtf_path)
        nav_entries = csv_data.csv_nav_entry.parse_file(csv_path)
        xtf_pings = csv_data.convert_matched_entries(xtf_pings, nav_entries)
        xtf_data.write_data(xtf_pings, "xtf_cache.cereal")
    return xtf_pings

gsf_pings = parse_or_load_gsf(sys.argv[1])
mbes_pings = gsf_data.convert_pings(gsf_pings)
xtf_pings = parse_or_load_xtf(sys.argv[2], sys.argv[3])

print "Time stamp of first xtf ping: ", xtf_pings[0].time_string_
print "Time stamp of last xtf ping: ", xtf_pings[-1].time_string_

d = draw_map.BathyMapImage(mbes_pings, 500, 500)
d.draw_height_map(mbes_pings)
d.draw_track(mbes_pings)
d.write_image("gsf_height_map.png")

sim_pings = []
for ping in xtf_pings:
    p = data_structures.mbes_ping()
    p.time_stamp_ = ping.time_stamp_
    p.time_string_ = ping.time_string_
    p.pos_ = ping.pos_
    sim_pings.append(p)

d = draw_map.BathyMapImage(mbes_pings, 500, 500)
d.draw_height_map(mbes_pings)
d.draw_track(sim_pings)
d.write_image("xtf_height_map.png")
d.show()
