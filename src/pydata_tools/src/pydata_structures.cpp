#include <data_tools/data_structures.h>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

namespace py = pybind11;

PYBIND11_MODULE(data_structures, m) {
    m.doc() = "Standard interfaces for working with different kinds of data. All data types should be converted into these before processing"; // optional module docstring

    py::class_<mbes_ping>(m, "mbes_ping", "Standard class interface for working with multibeam data")
        .def(py::init<>())
        .def_readwrite("id_", &mbes_ping::id_, "Member")
        .def_readwrite("time_string_", &mbes_ping::time_string_, "Member")
        .def_readwrite("time_stamp_", &mbes_ping::time_stamp_, "Member")
        .def_readwrite("heading_", &mbes_ping::heading_, "Member")
        .def_readwrite("heave_", &mbes_ping::heave_, "Member")
        .def_readwrite("pitch_", &mbes_ping::pitch_, "Member")
        .def_readwrite("roll_", &mbes_ping::roll_, "Member")
        .def_readwrite("first_in_file_", &mbes_ping::first_in_file_, "Member")
        .def_readwrite("pos_", &mbes_ping::pos_, "Member")
        .def_readwrite("beams", &mbes_ping::beams, "Member")
        .def_readwrite("back_scatter", &mbes_ping::back_scatter, "Member")
        .def_static("parse_file", &parse_file_from_str<mbes_ping>, "Parse mbes_ping from an ASCII file exported from NaviEdit")
        .def_static("parse_folder", &parse_folder_from_str<mbes_ping>, "Parse mbes_ping from folder of ASCII files exported from NaviEdit")
        .def_static("read_data", &read_data_from_str<mbes_ping::PingsT>, "Read mbes_ping::PingsT from .cereal file");

    py::class_<nav_entry>(m, "nav_entry", "Standard class interface for working with navigation data")
        .def(py::init<>())
        .def_readwrite("time_string_", &nav_entry::time_string_, "Member")
        .def_readwrite("time_stamp_", &nav_entry::time_stamp_, "Member")
        .def_readwrite("first_in_file_", &nav_entry::first_in_file_, "Member")
        .def_readwrite("pos_", &nav_entry::pos_, "Member")
        .def_static("parse_file", &parse_file_from_str<nav_entry>, "Parse nav_entry from an ASCII file exported from NaviEdit")
        .def_static("parse_folder", &parse_folder_from_str<nav_entry>, "Parse nav_entry from folder of ASCII files exported from NaviEdit")
        .def_static("read_data", &read_data_from_str<nav_entry::EntriesT>, "Read nav_entry::Entries from .cereal file");

    m.def("write_data", &write_data_from_str<mbes_ping::PingsT>, "Write mbes_ping::PingsT to .cereal file");
    m.def("write_data", &write_data_from_str<nav_entry::EntriesT>, "Write nav_entry::EntriesT to .cereal file");

}
