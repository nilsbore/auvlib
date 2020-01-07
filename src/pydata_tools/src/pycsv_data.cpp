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

#include <data_tools/csv_data.h>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

using namespace std_data;
using namespace csv_data;

namespace py = pybind11;

PYBIND11_MODULE(csv_data, m) {
    m.doc() = "Basic utilities for working with csv based data"; // optional module docstring

    py::class_<csv_nav_entry>(m, "csv_nav_entry", "Class for a csv based nav entry")
        .def(py::init<>())
        .def_readwrite("time_string_", &csv_nav_entry::time_string_, "Readable date of measurement")
        .def_readwrite("time_stamp_", &csv_nav_entry::time_stamp_, "UNIX timestamp")
        .def_readwrite("roll_", &csv_nav_entry::roll_, "Radian roll in ENU coordinates")
        .def_readwrite("pitch_", &csv_nav_entry::pitch_, "Radian pitch in ENU coordinates")
        .def_readwrite("heading_", &csv_nav_entry::heading_, "Radian yaw in ENU coordinates")
        .def_readwrite("roll_std_", &csv_nav_entry::roll_std_, "Roll standard deviation")
        .def_readwrite("pitch_std_", &csv_nav_entry::pitch_std_, "Pitch standard deviation")
        .def_readwrite("heading_std_", &csv_nav_entry::heading_std_, "Yaw standard deviation")
        .def_readwrite("lat_", &csv_nav_entry::lat_, "Latitude")
        .def_readwrite("long_", &csv_nav_entry::long_, "Longitude")
        .def_readwrite("altitude", &csv_nav_entry::altitude, "Altitude")
        .def_readwrite("pos_", &csv_nav_entry::pos_, "Position in ENU coordinates")
        .def_readwrite("vel_", &csv_nav_entry::vel_, "Velocity")
        .def_static("parse_file", &parse_file_from_str<csv_nav_entry>, "Parse csv_nav_entry from .csv file")
        .def_static("parse_folder", &parse_folder_from_str<csv_nav_entry>, "Parse csv_nav_entry from folder of .csv files")
        .def_static("read_data", &read_data_from_str<csv_nav_entry::EntriesT>, "Read csv_nav_entry::EntriesT from .cereal file");

    py::class_<csv_asvp_sound_speed>(m, "csv_asvp_sound_speed", "Class for a csv based nav entry")
        .def(py::init<>())
        .def_readwrite("time_string_", &csv_asvp_sound_speed::time_string_, "Readable date of measurement")
        .def_readwrite("time_stamp_", &csv_asvp_sound_speed::time_stamp_, "UNIX timestamp")
        .def_readwrite("lat_", &csv_asvp_sound_speed::lat_, "Latitude")
        .def_readwrite("long_", &csv_asvp_sound_speed::long_, "Longitude")
        .def_readwrite("dbars", &csv_asvp_sound_speed::dbars, "Pressure in decibars")
        .def_readwrite("vels", &csv_asvp_sound_speed::vels, "Corresponding velocities in m/s")
        .def_static("parse_file", &parse_file_from_str<csv_asvp_sound_speed>, "Parse csv_asvp_sound_speed from .csv file")
        .def_static("parse_folder", &parse_folder_from_str<csv_asvp_sound_speed>, "Parse csv_asvp_sound_speed from folder of .csv files")
        .def_static("read_data", &read_data_from_str<csv_asvp_sound_speed::EntriesT>, "Read csv_asvp_sound_speed::EntriesT from .cereal file");

    m.def("write_data", &write_data_from_str<csv_nav_entry::EntriesT>, "Write csv_nav_entry::EntriesT to .cereal file");
    m.def("write_data", &write_data_from_str<csv_asvp_sound_speed::EntriesT>, "Write csv_asvp_sound_speed::EntriesT to .cereal file");
    m.def("convert_matched_entries", (std_data::sss_ping::PingsT (*)(std_data::sss_ping::PingsT&, csv_nav_entry::EntriesT&) ) &convert_matched_entries, "Match xtf_sss_ping::PingsT and csv_nav_entry::EntriesT and assign pos info to pings");
}
