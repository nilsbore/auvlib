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

#include <data_tools/std_data.h>
#include <data_tools/navi_data.h>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

using namespace std_data;

namespace py = pybind11;

PYBIND11_MODULE(std_data, m) {
    m.doc() = "Standard interfaces for working with different kinds of data. All data types should be converted into these before processing"; // optional module docstring

    py::class_<mbes_ping>(m, "mbes_ping", "Standard class interface for working with multibeam data")
        .def(py::init<>())
        .def_readwrite("id_", &mbes_ping::id_, "Sequential ID of measurement")
        .def_readwrite("time_string_", &mbes_ping::time_string_, "Readable date of measurement")
        .def_readwrite("time_stamp_", &mbes_ping::time_stamp_, "UNIX timestamp")
        .def_readwrite("heading_", &mbes_ping::heading_, "Yaw in ENU coordinates")
        .def_readwrite("heave_", &mbes_ping::heave_, "Heave")
        .def_readwrite("pitch_", &mbes_ping::pitch_, "Radian pitch in ENU coordinates")
        .def_readwrite("roll_", &mbes_ping::roll_, "Radian roll in ENU coordinates")
        .def_readwrite("first_in_file_", &mbes_ping::first_in_file_, "Is first measurement in file?")
        .def_readwrite("pos_", &mbes_ping::pos_, "Position in ENU coordinates")
        .def_readwrite("beams", &mbes_ping::beams, "The beam hits in world ENU coordinates")
        .def_readwrite("back_scatter", &mbes_ping::back_scatter, "The beam reflectivities")
        .def_static("parse_file", &parse_file_from_str<mbes_ping>, "Parse mbes_ping from an ASCII file exported from NaviEdit")
        .def_static("parse_folder", &parse_folder_from_str<mbes_ping>, "Parse mbes_ping from folder of ASCII files exported from NaviEdit")
        .def_static("read_data", &read_data_from_str<mbes_ping::PingsT>, "Read mbes_ping::PingsT from .cereal file");

    py::class_<nav_entry>(m, "nav_entry", "Standard class interface for working with navigation data")
        .def(py::init<>())
        .def_readwrite("time_string_", &nav_entry::time_string_, "Readable date of measurement")
        .def_readwrite("time_stamp_", &nav_entry::time_stamp_, "UNIX timestamp")
        .def_readwrite("first_in_file_", &nav_entry::first_in_file_, "Is first measurement in file?")
        .def_readwrite("pos_", &nav_entry::pos_, "Position in ENU coordinates")
        .def_static("parse_file", &parse_file_from_str<nav_entry>, "Parse nav_entry from an ASCII file exported from NaviEdit")
        .def_static("parse_folder", &parse_folder_from_str<nav_entry>, "Parse nav_entry from folder of ASCII files exported from NaviEdit")
        .def_static("read_data", &read_data_from_str<nav_entry::EntriesT>, "Read nav_entry::Entries from .cereal file");

    py::class_<attitude_entry>(m, "attitude_entry", "Standard class interface for working with attitude data")
        .def(py::init<>())
        .def_readwrite("time_string_", &attitude_entry::time_string_, "Readable date of measurement")
        .def_readwrite("time_stamp_", &attitude_entry::time_stamp_, "UNIX timestamp")
        .def_readwrite("first_in_file_", &attitude_entry::first_in_file_, "Is first measurement in file?")
        .def_readwrite("roll", &attitude_entry::roll, "Radian roll in ENU coordinates")
        .def_readwrite("pitch", &attitude_entry::pitch, "Radian pitch in ENU coordinates")
        .def_readwrite("yaw", &attitude_entry::yaw, "Radian yaw in ENU coordinates")
        .def_readwrite("heave", &attitude_entry::heave, "Heave")
        .def_static("read_data", &read_data_from_str<attitude_entry::EntriesT>, "Read attitude_entry::Entries from .cereal file");

    m.def("write_data", &write_data_from_str<mbes_ping::PingsT>, "Write mbes_ping::PingsT to .cereal file");
    m.def("write_data", &write_data_from_str<nav_entry::EntriesT>, "Write nav_entry::EntriesT to .cereal file");
    m.def("write_data", &write_data_from_str<attitude_entry::EntriesT>, "Write attitude_entry::EntriesT to .cereal file");

}
