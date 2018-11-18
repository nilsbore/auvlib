#include <data_tools/xtf_data.h>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

namespace py = pybind11;

PYBIND11_MODULE(pyxtf_data, m) {
    m.doc() = "Basic utilities for reading xtf based data"; // optional module docstring

    py::class_<xtf_sss_ping_side>(m, "xtf_sss_ping_side")
        .def(py::init<>())
        .def_readwrite("pings", &xtf_sss_ping_side::pings)
        .def_readwrite("slant_range", &xtf_sss_ping_side::slant_range)
        .def_readwrite("time_duration", &xtf_sss_ping_side::time_duration)
        .def_readwrite("tilt_angle", &xtf_sss_ping_side::tilt_angle)
        .def_readwrite("beam_width", &xtf_sss_ping_side::beam_width);

    py::class_<xtf_sss_ping>(m, "xtf_sss_ping")
        .def(py::init<>())
        .def_readwrite("time_string_", &xtf_sss_ping::time_string_)
        .def_readwrite("time_stamp_", &xtf_sss_ping::time_stamp_)
        .def_readwrite("first_in_file_", &xtf_sss_ping::first_in_file_)
        .def_readwrite("heading_", &xtf_sss_ping::heading_)
        .def_readwrite("pitch_", &xtf_sss_ping::pitch_)
        .def_readwrite("roll_", &xtf_sss_ping::roll_)
        .def_readwrite("lat_", &xtf_sss_ping::lat_)
        .def_readwrite("long_", &xtf_sss_ping::long_)
        .def_readwrite("sound_vel_", &xtf_sss_ping::sound_vel_)
        .def_readwrite("pos_", &xtf_sss_ping::pos_);

    m.def("parse_file", &parse_file_from_str<xtf_sss_ping>, "A function which parses xtf sss pings");
    m.def("parse_folder", &parse_folder_from_str<xtf_sss_ping>, "A function which parses xtf sss pings from folder");
    m.def("read_data", &read_data_from_str<xtf_sss_ping::PingsT>, "A function which reads xtf pings from a .cereal file");
    m.def("write_data", &write_data_from_str<xtf_sss_ping::PingsT>, "A function which writes xtf pings to a .cereal file");
    m.def("make_waterfall_image", &make_waterfall_image, "A function which creates and opencv waterfall image");
}
