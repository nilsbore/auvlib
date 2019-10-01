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

#include <data_tools/gsf_data.h>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

using namespace std_data;
using namespace gsf_data;

namespace py = pybind11;

/*
 * Screw this for now. Idea would be to instantiate a class
 * and automatically get the member pointer from the instance
 * by subtracting the class offset from the pointer. if pybind
 * accepts merely the type pointer (as opposed to member pointer
 * this should be fine. But it's a bit to hacky and risky for now
 */

/*
template <typename T>
class PybindCerealArchive {

private:

    py::class_<T>& c;

public:

    PybindCerealArchive(py::class_<T>& c) : c(c) {}

    template <typename A>
    void bind_values(A&& key_value)
    {
        c.def_readwrite(key_value.name, key_value.value);
    }

    template<typename ... Args>
    void operator()(Args&& ... args)
    {
        bind_values(args)...;
    }

};
*/

PYBIND11_MODULE(gsf_data, m) {
    m.doc() = "Basic utilities for working with the .gsf file format"; // optional module docstring

    py::class_<gsf_mbes_ping>(m, "gsf_mbes_ping", "Class for the gsf multibeam type")
        .def(py::init<>())
        .def_readwrite("time_string_", &gsf_mbes_ping::time_string_, "Readable date of measurement")
        .def_readwrite("time_stamp_", &gsf_mbes_ping::time_stamp_, "UNIX timestamp")
        .def_readwrite("travel_times", &gsf_mbes_ping::travel_times, "Travel times of beam returns")
        .def_readwrite("beam_angles", &gsf_mbes_ping::beam_angles, "Angles of beams")
        .def_readwrite("distances", &gsf_mbes_ping::distances, "Distances of beam returns")
        .def_readwrite("amplitudes", &gsf_mbes_ping::amplitudes, "Intensities of beam returns")
        .def_readwrite("first_in_file_", &gsf_mbes_ping::first_in_file_, "Is first measurement in file?")
        .def_readwrite("heading_", &gsf_mbes_ping::heading_, "Radian yaw in ENU coordinates")
        .def_readwrite("pitch_", &gsf_mbes_ping::pitch_, "Radian pitch in ENU coordinates")
        .def_readwrite("roll_", &gsf_mbes_ping::roll_, "Radian roll in ENU coordinates")
        .def_readwrite("lat_", &gsf_mbes_ping::lat_, "Latitude")
        .def_readwrite("long_", &gsf_mbes_ping::long_, "Longitude")
        .def_readwrite("depth_", &gsf_mbes_ping::depth_, "Depth")
        .def_readwrite("beams", &gsf_mbes_ping::beams, "The hit positions in vehicle coordinates")
        .def_static("parse_file", &parse_file_from_str<gsf_mbes_ping>, "Parse gsf_mbes_ping from .gsf file")
        .def_static("parse_folder", &parse_folder_from_str<gsf_mbes_ping>, "Parse gsf_mbes_ping from folder of .gsf files")
        .def_static("read_data", &read_data_from_str<gsf_mbes_ping::PingsT>, "Read gsf_mbes_ping::PingsT from .cereal file");
    //PybindCerealArchive<gsf_mbes_ping> archive(c);
    //gsf_mbes_ping example; example.serialize(archive);

    py::class_<gsf_sound_speed>(m, "gsf_sound_speed", "Class for the gsf sound speed type")
        .def(py::init<>())
        .def_readwrite("time_string_", &gsf_sound_speed::time_string_, "Readable date of measurement")
        .def_readwrite("time_stamp_", &gsf_sound_speed::time_stamp_, "UNIX timestamp")
        .def_readwrite("near_speed", &gsf_sound_speed::near_speed, "Sound speed at vehicle depth")
        .def_readwrite("below_speed", &gsf_sound_speed::below_speed, "Sound speed below vehicle")
        .def_static("parse_file", &parse_file_from_str<gsf_sound_speed>, "Parse gsf_sound_speed from .gsf file")
        .def_static("parse_folder", &parse_folder_from_str<gsf_sound_speed>, "Parse gsf_sound_speed from folder of .gsf files")
        .def_static("read_data", &read_data_from_str<gsf_sound_speed::SpeedsT>, "Read gsf_sound_speed::SpeedsT from .cereal file");

    py::class_<gsf_nav_entry>(m, "gsf_nav_entry", "Class for the gsf nav entry type")
        .def(py::init<>())
        .def_readwrite("id_", &gsf_nav_entry::id_, "Sequential ID of measurement")
        .def_readwrite("time_string_", &gsf_nav_entry::time_string_, "Readable date of measurement")
        .def_readwrite("time_stamp_", &gsf_nav_entry::time_stamp_, "UNIX timestamp")
        .def_readwrite("roll_", &gsf_nav_entry::roll_, "Radian roll in ENU coordinates")
        .def_readwrite("pitch_", &gsf_nav_entry::pitch_, "Radian pitch in ENU coordinates")
        .def_readwrite("yaw_", &gsf_nav_entry::yaw_, "Radian yaw in ENU coordinates")
        .def_readwrite("lat_", &gsf_nav_entry::lat_, "Latitude")
        .def_readwrite("long_", &gsf_nav_entry::long_, "Longitude")
        .def_readwrite("altitude", &gsf_nav_entry::altitude, "Altitude")
        .def_readwrite("pos_", &gsf_nav_entry::pos_, "Position in ENU coordinates")
        .def_static("parse_file", &parse_file_from_str<gsf_nav_entry>, "Parse gsf_nav_entry from .gsf file")
        .def_static("parse_folder", &parse_folder_from_str<gsf_nav_entry>, "Parse gsf_nav_entry from folder of .gsf files")
        .def_static("read_data", &read_data_from_str<gsf_nav_entry::EntriesT>, "Read gsf_nav_entry::EntriesT from .cereal file");

    m.def("write_data", &write_data_from_str<gsf_mbes_ping::PingsT>, "Write gsf_mbes_ping::PingsT to .cereal file");
    m.def("write_data", &write_data_from_str<gsf_nav_entry::EntriesT>, "Write gsf_nav_entry::EntriesT to .cereal file");
    m.def("write_data", &write_data_from_str<gsf_sound_speed::SpeedsT>, "Write gsf_sound_speed::SpeedsT to .cereal file");
    m.def("convert_pings", &convert_pings, "Convert gsf_mbes_ping::EntriesT to mbes_ping::EntriesT");
}
