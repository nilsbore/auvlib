from auvlib.data_tools import std_data, gsf_data, csv_data, xtf_data
import os

def parse_or_load_gsf(path):

    if os.path.exists("gsf_cache.cereal"):
        gsf_pings = gsf_data.gsf_mbes_ping.read_data("gsf_cache.cereal")
    else:
        gsf_pings = gsf_data.gsf_mbes_ping.parse_folder(path)
        gsf_data.write_data(gsf_pings, "gsf_cache.cereal")
    return gsf_pings

def parse_or_load_csv(path):

    if os.path.exists("csv_cache.cereal"):
        entries = csv_data.csv_nav_entry.read_data("csv_cache.cereal")
    else:
        entries = csv_data.csv_nav_entry.parse_file(path)
        csv_data.write_data(entries, "csv_cache.cereal")
    return entries

def parse_or_load_xtf(path):

    if os.path.exists("xtf_cache.cereal"):
        xtf_pings = xtf_data.xtf_sss_ping.read_data("xtf_cache.cereal")
    else:
        xtf_pings = xtf_data.xtf_sss_ping.parse_folder(path)
        #nav_entries = csv_data.csv_nav_entry.parse_file(csv_path)
        xtf_data.write_data(xtf_pings, "xtf_cache.cereal")
    return xtf_pings
