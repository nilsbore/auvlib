#include <data_tools/csv_data.h>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

namespace py = pybind11;

PYBIND11_MODULE(csv_data, m) {
    m.doc() = "Basic utilities for reading csv based data"; // optional module docstring

    py::class_<csv_nav_entry>(m, "csv_nav_entry")
        .def(py::init<>())
        .def_readwrite("time_string_", &csv_nav_entry::time_string_)
        .def_readwrite("time_stamp_", &csv_nav_entry::time_stamp_)
        .def_readwrite("roll_", &csv_nav_entry::roll_)
        .def_readwrite("pitch_", &csv_nav_entry::pitch_)
        .def_readwrite("heading_", &csv_nav_entry::heading_)
        .def_readwrite("roll_std_", &csv_nav_entry::roll_std_)
        .def_readwrite("pitch_std_", &csv_nav_entry::pitch_std_)
        .def_readwrite("heading_std_", &csv_nav_entry::heading_std_)
        .def_readwrite("lat_", &csv_nav_entry::lat_)
        .def_readwrite("long_", &csv_nav_entry::long_)
        .def_readwrite("altitude", &csv_nav_entry::altitude)
        .def_readwrite("pos_", &csv_nav_entry::pos_)
        .def_readwrite("vel_", &csv_nav_entry::pos_);

    m.def("parse_file", &parse_file_from_str<csv_nav_entry>, "A function which parses xtf sss pings");
    m.def("parse_folder", &parse_folder_from_str<csv_nav_entry>, "A function which parses xtf sss pings from folder");
    m.def("convert_matched_entries", (xtf_sss_ping::PingsT (*)(xtf_sss_ping::PingsT&, csv_nav_entry::EntriesT&) ) &convert_matched_entries, "A function which matches xtf pings and csv nav entries");
}
