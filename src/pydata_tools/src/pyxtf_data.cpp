#include <data_tools/xtf_data.h>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

using namespace data_structures;
using namespace xtf_data;

namespace py = pybind11;

PYBIND11_MODULE(xtf_data, m) {
    m.doc() = "Basic utilities for working with the xtf file format"; // optional module docstring

    py::class_<xtf_sss_ping_side>(m, "xtf_sss_ping_side", "Class for one xtf sidescan side")
        .def(py::init<>())
        .def_readwrite("pings", &xtf_sss_ping_side::pings, "Member")
        .def_readwrite("slant_range", &xtf_sss_ping_side::slant_range, "Member")
        .def_readwrite("time_duration", &xtf_sss_ping_side::time_duration, "Member")
        .def_readwrite("tilt_angle", &xtf_sss_ping_side::tilt_angle, "Member")
        .def_readwrite("beam_width", &xtf_sss_ping_side::beam_width, "Member");

    py::class_<xtf_sss_ping>(m, "xtf_sss_ping", "Class for xtf sidescan type")
        .def(py::init<>())
        .def_readwrite("time_string_", &xtf_sss_ping::time_string_, "Member")
        .def_readwrite("time_stamp_", &xtf_sss_ping::time_stamp_, "Member")
        .def_readwrite("port", &xtf_sss_ping::port, "Member")
        .def_readwrite("stbd", &xtf_sss_ping::stbd, "Member")
        .def_readwrite("first_in_file_", &xtf_sss_ping::first_in_file_, "Member")
        .def_readwrite("heading_", &xtf_sss_ping::heading_, "Member")
        .def_readwrite("pitch_", &xtf_sss_ping::pitch_, "Member")
        .def_readwrite("roll_", &xtf_sss_ping::roll_, "Member")
        .def_readwrite("lat_", &xtf_sss_ping::lat_, "Member")
        .def_readwrite("long_", &xtf_sss_ping::long_, "Member")
        .def_readwrite("sound_vel_", &xtf_sss_ping::sound_vel_, "Member")
        .def_readwrite("pos_", &xtf_sss_ping::pos_, "Member")
        .def_static("parse_file", &parse_file_from_str<xtf_sss_ping>, "Parse xtf_sss_ping from .xtf file")
        .def_static("parse_folder", &parse_folder_from_str<xtf_sss_ping>, "Parse xtf_sss_ping from folder of .xtf files")
        .def_static("read_data", &read_data_from_str<xtf_sss_ping::PingsT>, "Read xtf_sss_ping::PingsT from .cereal file");

    m.def("write_data", &write_data_from_str<xtf_sss_ping::PingsT>, "Write xtf pings to .cereal file");
    m.def("make_waterfall_image", &make_waterfall_image, "Create an opencv waterfall image from xtf_sss_ping::PingsT");
    m.def("correct_sensor_offset", &correct_sensor_offset, "Move the sensor onboard the vehicle with a given translation");
}
