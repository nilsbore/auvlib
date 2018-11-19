#include <data_tools/data_structures.h>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

namespace py = pybind11;

PYBIND11_MODULE(data_structures, m) {
    m.doc() = "Basic data structures for sonar data"; // optional module docstring

    py::class_<mbes_ping>(m, "mbes_ping")
        .def(py::init<>())
        .def_readwrite("id_", &mbes_ping::id_)
        .def_readwrite("time_string_", &mbes_ping::time_string_)
        .def_readwrite("time_stamp_", &mbes_ping::time_stamp_)
        .def_readwrite("heading_", &mbes_ping::heading_)
        .def_readwrite("heave_", &mbes_ping::heave_)
        .def_readwrite("pitch_", &mbes_ping::pitch_)
        .def_readwrite("roll_", &mbes_ping::roll_)
        .def_readwrite("first_in_file_", &mbes_ping::first_in_file_)
        .def_readwrite("pos_", &mbes_ping::pos_)
        .def_readwrite("beams", &mbes_ping::beams)
        .def_readwrite("back_scatter", &mbes_ping::back_scatter)
        .def_static("parse_file", &parse_file_from_str<mbes_ping>)
        .def_static("parse_folder", &parse_folder_from_str<mbes_ping>)
        .def_static("read_data", &read_data_from_str<mbes_ping::PingsT>);

    py::class_<nav_entry>(m, "nav_entry")
        .def(py::init<>())
        .def_readwrite("time_string_", &nav_entry::time_string_)
        .def_readwrite("time_stamp_", &nav_entry::time_stamp_)
        .def_readwrite("first_in_file_", &nav_entry::first_in_file_)
        .def_readwrite("pos_", &nav_entry::pos_)
        .def_static("parse_file", &parse_file_from_str<nav_entry>)
        .def_static("parse_folder", &parse_folder_from_str<nav_entry>)
        .def_static("read_data", &read_data_from_str<nav_entry::EntriesT>);

    m.def("write_data", &write_data_from_str<mbes_ping::PingsT>, "A function which writes mbes pings to a .cereal file");
    m.def("write_data", &write_data_from_str<nav_entry::EntriesT>, "A function which writes nav entries to a .cereal file");

}
