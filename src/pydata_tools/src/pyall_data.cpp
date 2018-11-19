#include <data_tools/all_data.h>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

namespace py = pybind11;

PYBIND11_MODULE(all_data, m) {
    m.doc() = "Basic data structures for sonar data"; // optional module docstring

    py::class_<all_mbes_ping>(m, "all_mbes_ping")
        .def(py::init<>())
        .def_readwrite("id_", &all_mbes_ping::id_)
        .def_readwrite("time_string_", &all_mbes_ping::time_string_)
        .def_readwrite("time_stamp_", &all_mbes_ping::time_stamp_)
        .def_readwrite("heading_", &all_mbes_ping::heading_)
        .def_readwrite("sound_vel_", &all_mbes_ping::sound_vel_)
        .def_readwrite("transducer_depth_", &all_mbes_ping::transducer_depth_)
        .def_readwrite("reflectivities", &all_mbes_ping::reflectivities)
        .def_readwrite("beams", &all_mbes_ping::beams)
        .def_readwrite("first_in_file_", &all_mbes_ping::first_in_file_)
        .def_static("parse_file", &parse_file_from_str<all_mbes_ping>)
        .def_static("parse_folder", &parse_folder_from_str<all_mbes_ping>)
        .def_static("read_data", &read_data_from_str<all_mbes_ping::PingsT>);

    py::class_<all_nav_entry>(m, "all_nav_entry")
        .def(py::init<>())
        .def_readwrite("id_", &all_nav_entry::id_)
        .def_readwrite("time_string_", &all_nav_entry::time_string_)
        .def_readwrite("time_stamp_", &all_nav_entry::time_stamp_)
        .def_readwrite("lat_", &all_nav_entry::lat_)
        .def_readwrite("long_", &all_nav_entry::long_)
        .def_readwrite("depth_", &all_nav_entry::depth_)
        .def_readwrite("heading_", &all_nav_entry::heading_)
        .def_readwrite("course_over_ground_", &all_nav_entry::course_over_ground_)
        .def_readwrite("first_in_file_", &all_nav_entry::first_in_file_)
        .def_static("parse_file", &parse_file_from_str<all_nav_entry>)
        .def_static("parse_folder", &parse_folder_from_str<all_nav_entry>)
        .def_static("read_data", &read_data_from_str<all_nav_entry::EntriesT>);

    py::class_<all_nav_depth>(m, "all_nav_depth")
        .def(py::init<>())
        .def_readwrite("id_", &all_nav_depth::id_)
        .def_readwrite("time_string_", &all_nav_depth::time_string_)
        .def_readwrite("time_stamp_", &all_nav_depth::time_stamp_)
        .def_readwrite("height", &all_nav_depth::height)
        .def_readwrite("height_type", &all_nav_depth::height_type)
        .def_readwrite("first_in_file_", &all_nav_depth::first_in_file_)
        .def_static("parse_file", &parse_file_from_str<all_nav_depth>)
        .def_static("parse_folder", &parse_folder_from_str<all_nav_depth>)
        .def_static("read_data", &read_data_from_str<all_nav_depth::EntriesT>);

    py::class_<all_echosounder_depth>(m, "all_echosounder_depth")
        .def(py::init<>())
        .def_readwrite("id_", &all_echosounder_depth::id_)
        .def_readwrite("time_string_", &all_echosounder_depth::time_string_)
        .def_readwrite("time_stamp_", &all_echosounder_depth::time_stamp_)
        .def_readwrite("depth_", &all_echosounder_depth::depth_)
        .def_readwrite("first_in_file_", &all_echosounder_depth::first_in_file_)
        .def_static("parse_file", &parse_file_from_str<all_echosounder_depth>)
        .def_static("parse_folder", &parse_folder_from_str<all_echosounder_depth>)
        .def_static("read_data", &read_data_from_str<all_echosounder_depth::EntriesT>);

    m.def("write_data", &write_data_from_str<all_mbes_ping::PingsT>, "A function which writes all mbes pings to a .cereal file");
    m.def("write_data", &write_data_from_str<all_nav_entry::EntriesT>, "A function which writes all nav entries to a .cereal file");
    m.def("write_data", &write_data_from_str<all_nav_depth::EntriesT>, "A function which writes all nav depths to a .cereal file");
    m.def("write_data", &write_data_from_str<all_echosounder_depth::EntriesT>, "A function which writes all echosounder depths to a .cereal file");
    m.def("convert_matched_entries", (all_mbes_ping::PingsT (*)(all_mbes_ping::PingsT&, all_nav_entry::EntriesT&) ) &convert_matched_entries, "A function which matches xtf pings and csv nav entries");
}
