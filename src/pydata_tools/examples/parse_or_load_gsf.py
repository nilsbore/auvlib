#!/usr/bin/python

from auvlib.data_tools import std_data, gsf_data
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

for ping in mbes_pings:
    print "Ping pos: ", ping.pos_.transpose()
