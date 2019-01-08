#!/usr/bin/python

from auvlib.data_tools import std_data, gsf_data, xtf_data, csv_data
from auvlib.bathy_maps import draw_map, mesh_map, patch_draper
import sys
import os

gsf_pings = utils.parse_or_load_gsf(sys.argv[1])
mbes_pings = utils.gsf_data.convert_pings(gsf_pings)
xtf_pings = utils.parse_or_load_xtf(sys.argv[2])
nav_entries = utils.parse_or_load_csv(sys.argv[3])
xtf_pings = csv_data.convert_matched_entries(xtf_pings, nav_entries)

print "Time stamp of first xtf ping: ", xtf_pings[0].time_string_
print "Time stamp of last xtf ping: ", xtf_pings[-1].time_string_

d = draw_map.BathyMapImage(mbes_pings, 500, 500)
d.draw_height_map(mbes_pings)
d.draw_track(mbes_pings)
d.write_image("gsf_height_map.png")

sim_pings = []
for ping in xtf_pings:
    p = std_data.mbes_ping()
    p.time_stamp_ = ping.time_stamp_
    p.time_string_ = ping.time_string_
    p.pos_ = ping.pos_
    sim_pings.append(p)

d = draw_map.BathyMapImage(mbes_pings, 500, 500)
d.draw_height_map(mbes_pings)
d.draw_track(sim_pings)
d.write_image("xtf_height_map.png")
d.show()
