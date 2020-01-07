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

#include <data_tools/jsf_data.h>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

using namespace std_data;
using namespace jsf_data;

namespace py = pybind11;

PYBIND11_MODULE(jsf_data, m) {
    m.doc() = "Basic utilities for working with the jsf file format"; // optional module docstring

    py::class_<jsf_sss_ping_side>(m, "jsf_sss_ping_side", "Class for one jsf sidescan side")
        .def(py::init<>())
        .def_readwrite("pings", &jsf_sss_ping_side::pings, "The return intensities")
        .def_readwrite("pings_phase", &jsf_sss_ping_side::pings_phase, "The return intensities")
        .def_readwrite("time_duration", &jsf_sss_ping_side::time_duration, "Furthest measured intensity");

    py::class_<jsf_sss_ping>(m, "jsf_sss_ping", "Class for jsf sidescan type")
        .def(py::init<>())
        .def_readwrite("time_string_", &jsf_sss_ping::time_string_, "Readable date of measurement")
        .def_readwrite("time_stamp_", &jsf_sss_ping::time_stamp_, "UNIX timestamp")
        .def_readwrite("port", &jsf_sss_ping::port, "Port measurement")
        .def_readwrite("stbd", &jsf_sss_ping::stbd, "Starboard measurement")
        .def_readwrite("first_in_file_", &jsf_sss_ping::first_in_file_, "Is first measurement in file?")
        .def_readwrite("rpy", &jsf_sss_ping::rpy, "Radian roll, pitch, yaw in ENU coordinates")
        .def_readwrite("lat_", &jsf_sss_ping::lat_, "Latitude")
        .def_readwrite("long_", &jsf_sss_ping::long_, "Longitude")
        .def_readwrite("utm_zone", &jsf_sss_ping::utm_zone, "Identifier of the UTM zone of coordinates in pos vector")
        .def_readwrite("sound_vel_", &jsf_sss_ping::sound_vel, "Sound speed in m/s")
        .def_readwrite("frequency", &jsf_sss_ping::frequency, "Frequency of sampling")
        .def_readwrite("pos_", &jsf_sss_ping::pos_, "Position in ENU coordinates")
        .def_static("parse_file", &parse_file_from_str<jsf_sss_ping>, "Parse jsf_sss_ping from .jsf file")
        .def_static("parse_folder", &parse_folder_from_str<jsf_sss_ping>, "Parse jsf_sss_ping from folder of .jsf files")
        .def_static("read_data", &read_data_from_str<jsf_sss_ping::PingsT>, "Read jsf_sss_ping::PingsT from .cereal file");

    m.def("write_data", &write_data_from_str<jsf_sss_ping::PingsT>, "Write jsf pings to .cereal file");
    m.def("make_waterfall_image", &make_waterfall_image, "Create a cv2 waterfall image from jsf_sss_ping::PingsT");
    m.def("show_waterfall_image", &show_waterfall_image, "Show a waterfall image created from jsf_sss_ping::PingsT");
    m.def("filter_frequency", &filter_frequency, "Filter to keep only jsf_sss_ping::PingsT with certain frequency");
    m.def("convert_to_xtf_pings", &convert_to_xtf_pings, "Convert jsf_sss_ping::PingsT to std_data::sss_ping::PingsT");

    // from http://alexsm.com/pybind11-buffer-protocol-opencv-to-numpy/
    pybind11::class_<cv::Mat>(m, "Image", pybind11::buffer_protocol())
        .def_buffer([](cv::Mat& im) -> pybind11::buffer_info {
            return pybind11::buffer_info(
                // Pointer to buffer
                im.data,
                // Size of one scalar
                sizeof(unsigned char),
                // Python struct-style format descriptor
                pybind11::format_descriptor<unsigned char>::format(),
                // Number of dimensions
                3,
                // Buffer dimensions
                { im.rows, im.cols, im.channels() },
                // Strides (in bytes) for each index
                {
                    sizeof(unsigned char) * im.channels() * im.cols,
                    sizeof(unsigned char) * im.channels(),
                    sizeof(unsigned char)
                }
            );
        });
}
