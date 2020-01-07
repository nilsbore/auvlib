/* Copyright 2018 Nils Bore (nbore@kth.se)
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <data_tools/xtf_data.h>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

using namespace std_data;
using namespace xtf_data;

namespace py = pybind11;

PYBIND11_MODULE(xtf_data, m) {
    m.doc() = "Basic utilities for working with the xtf file format"; // optional module docstring

    py::class_<xtf_sss_ping_side>(m, "xtf_sss_ping_side", py::module_local(), "Class for one xtf sidescan side")
        .def(py::init<>())
        .def_readwrite("pings", &xtf_sss_ping_side::pings, "The return intensities")
        .def_readwrite("slant_range", &xtf_sss_ping_side::slant_range, "Slant range")
        .def_readwrite("time_duration", &xtf_sss_ping_side::time_duration, "Furthest measured intensity")
        .def_readwrite("tilt_angle", &xtf_sss_ping_side::tilt_angle, "Tilt angle")
        .def_readwrite("beam_width", &xtf_sss_ping_side::beam_width, "Beam width");

    py::class_<xtf_sss_ping>(m, "xtf_sss_ping", py::module_local(), "Class for xtf sidescan type")
        .def(py::init<>())
        .def_readwrite("time_string_", &xtf_sss_ping::time_string_, "Readable date of measurement")
        .def_readwrite("time_stamp_", &xtf_sss_ping::time_stamp_, "UNIX timestamp")
        .def_readwrite("port", &xtf_sss_ping::port, "Port measurement")
        .def_readwrite("stbd", &xtf_sss_ping::stbd, "Starboard measurement")
        .def_readwrite("first_in_file_", &xtf_sss_ping::first_in_file_, "Is first measurement in file?")
        .def_readwrite("heading_", &xtf_sss_ping::heading_, "Radian yaw in ENU coordinates")
        .def_readwrite("pitch_", &xtf_sss_ping::pitch_, "Radian pitch in ENU coordinates")
        .def_readwrite("roll_", &xtf_sss_ping::roll_, "Radian roll in ENU coordinates")
        .def_readwrite("lat_", &xtf_sss_ping::lat_, "Latitude")
        .def_readwrite("long_", &xtf_sss_ping::long_, "Longitude")
        .def_readwrite("sound_vel_", &xtf_sss_ping::sound_vel_, "Sound speed in m/s")
        .def_readwrite("pos_", &xtf_sss_ping::pos_, "Position in ENU coordinates")
        .def_static("parse_file", &parse_file_from_str<xtf_sss_ping>, "Parse xtf_sss_ping from .xtf file")
        .def_static("parse_folder", &parse_folder_from_str<xtf_sss_ping>, "Parse xtf_sss_ping from folder of .xtf files")
        .def_static("read_data", &read_data_from_str<xtf_sss_ping::PingsT>, "Read xtf_sss_ping::PingsT from .cereal file");

    m.def("write_data", &write_data_from_str<xtf_sss_ping::PingsT>, "Write xtf pings to .cereal file");
    m.def("make_waterfall_image", &make_eigen_waterfall_image, "Create a cv2 waterfall image from xtf_sss_ping::PingsT");
    m.def("show_waterfall_image", &show_waterfall_image, "Show a waterfall image created from xtf_sss_ping::PingsT");
    m.def("correct_sensor_offset", &correct_sensor_offset, "Move the sensor onboard the vehicle with a given translation");
    m.def("match_attitudes", &match_attitudes, "Get roll and pitch from std_data::attitude_entry by matching timestamps");
}
