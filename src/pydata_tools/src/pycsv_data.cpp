#include <data_tools/csv_data.h>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

namespace py = pybind11;

PYBIND11_MODULE(csv_data, m) {
    m.doc() = "Basic utilities for working with csv based data"; // optional module docstring

    py::class_<csv_nav_entry>(m, "csv_nav_entry", "Class for a csv based nav entry")
        .def(py::init<>())
        .def_readwrite("time_string_", &csv_nav_entry::time_string_, "Member")
        .def_readwrite("time_stamp_", &csv_nav_entry::time_stamp_, "Member")
        .def_readwrite("roll_", &csv_nav_entry::roll_, "Member")
        .def_readwrite("pitch_", &csv_nav_entry::pitch_, "Member")
        .def_readwrite("heading_", &csv_nav_entry::heading_, "Member")
        .def_readwrite("roll_std_", &csv_nav_entry::roll_std_, "Member")
        .def_readwrite("pitch_std_", &csv_nav_entry::pitch_std_, "Member")
        .def_readwrite("heading_std_", &csv_nav_entry::heading_std_, "Member")
        .def_readwrite("lat_", &csv_nav_entry::lat_, "Member")
        .def_readwrite("long_", &csv_nav_entry::long_, "Member")
        .def_readwrite("altitude", &csv_nav_entry::altitude, "Member")
        .def_readwrite("pos_", &csv_nav_entry::pos_, "Member")
        .def_readwrite("vel_", &csv_nav_entry::vel_, "Member")
        .def_static("parse_file", &parse_file_from_str<csv_nav_entry>, "Parse csv_nav_entry from .csv file")
        .def_static("parse_folder", &parse_folder_from_str<csv_nav_entry>, "Parse csv_nav_entry from folder of .csv files")
        .def_static("read_data", &read_data_from_str<csv_nav_entry::EntriesT>, "Read csv_nav_entry::EntriesT from .cereal file");

    py::class_<csv_asvp_sound_speed>(m, "csv_asvp_sound_speed", "Class for a csv based nav entry")
        .def(py::init<>())
        .def_readwrite("time_string_", &csv_asvp_sound_speed::time_string_, "Member")
        .def_readwrite("time_stamp_", &csv_asvp_sound_speed::time_stamp_, "Member")
        .def_readwrite("lat_", &csv_asvp_sound_speed::lat_, "Member")
        .def_readwrite("long_", &csv_asvp_sound_speed::long_, "Member")
        .def_readwrite("dbars", &csv_asvp_sound_speed::dbars, "Member")
        .def_readwrite("vels", &csv_asvp_sound_speed::vels, "Member")
        .def_static("parse_file", &parse_file_from_str<csv_asvp_sound_speed>, "Parse csv_asvp_sound_speed from .csv file")
        .def_static("parse_folder", &parse_folder_from_str<csv_asvp_sound_speed>, "Parse csv_asvp_sound_speed from folder of .csv files")
        .def_static("read_data", &read_data_from_str<csv_asvp_sound_speed::EntriesT>, "Read csv_asvp_sound_speed::EntriesT from .cereal file");

    m.def("write_data", &write_data_from_str<csv_nav_entry::EntriesT>, "Write csv_nav_entry::EntriesT to .cereal file");
    m.def("write_data", &write_data_from_str<csv_asvp_sound_speed::EntriesT>, "Write csv_asvp_sound_speed::EntriesT to .cereal file");
    m.def("convert_matched_entries", (xtf_sss_ping::PingsT (*)(xtf_sss_ping::PingsT&, csv_nav_entry::EntriesT&) ) &convert_matched_entries, "Match xtf_sss_ping::PingsT and csv_nav_entry::EntriesT and assign pos info to pings");
}
