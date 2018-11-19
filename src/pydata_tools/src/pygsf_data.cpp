#include <data_tools/gsf_data.h>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

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
    m.doc() = "Basic data structures for sonar data"; // optional module docstring

    py::class_<gsf_mbes_ping>(m, "gsf_mbes_ping")
        .def(py::init<>())
        .def_readwrite("time_string_", &gsf_mbes_ping::time_string_)
        .def_readwrite("time_stamp_", &gsf_mbes_ping::time_stamp_)
        .def_readwrite("travel_times", &gsf_mbes_ping::travel_times)
        .def_readwrite("beam_angles", &gsf_mbes_ping::beam_angles)
        .def_readwrite("distances", &gsf_mbes_ping::distances)
        .def_readwrite("amplitudes", &gsf_mbes_ping::amplitudes)
        .def_readwrite("first_in_file_", &gsf_mbes_ping::first_in_file_)
        .def_readwrite("heading_", &gsf_mbes_ping::heading_)
        .def_readwrite("pitch_", &gsf_mbes_ping::pitch_)
        .def_readwrite("roll_", &gsf_mbes_ping::roll_)
        .def_readwrite("lat_", &gsf_mbes_ping::lat_)
        .def_readwrite("long_", &gsf_mbes_ping::long_)
        .def_readwrite("depth_", &gsf_mbes_ping::depth_)
        .def_readwrite("beams", &gsf_mbes_ping::beams)
        .def_static("parse_file", &parse_file_from_str<gsf_mbes_ping>)
        .def_static("parse_folder", &parse_folder_from_str<gsf_mbes_ping>)
        .def_static("read_data", &read_data_from_str<gsf_mbes_ping::PingsT>);
    //PybindCerealArchive<gsf_mbes_ping> archive(c);
    //gsf_mbes_ping example; example.serialize(archive);

    py::class_<gsf_sound_speed>(m, "gsf_sound_speed")
        .def(py::init<>())
        .def_readwrite("time_string_", &gsf_sound_speed::time_string_)
        .def_readwrite("time_stamp_", &gsf_sound_speed::time_stamp_)
        .def_readwrite("near_speed", &gsf_sound_speed::near_speed)
        .def_readwrite("below_speed", &gsf_sound_speed::below_speed)
        .def_static("parse_file", &parse_file_from_str<gsf_sound_speed>)
        .def_static("parse_folder", &parse_folder_from_str<gsf_sound_speed>)
        .def_static("read_data", &read_data_from_str<gsf_sound_speed::SpeedsT>);

    py::class_<gsf_nav_entry>(m, "gsf_nav_entry")
        .def(py::init<>())
        .def_readwrite("id_", &gsf_nav_entry::id_)
        .def_readwrite("time_string_", &gsf_nav_entry::time_string_)
        .def_readwrite("time_stamp_", &gsf_nav_entry::time_stamp_)
        .def_readwrite("roll_", &gsf_nav_entry::roll_)
        .def_readwrite("pitch_", &gsf_nav_entry::pitch_)
        .def_readwrite("yaw_", &gsf_nav_entry::yaw_)
        .def_readwrite("lat_", &gsf_nav_entry::lat_)
        .def_readwrite("long_", &gsf_nav_entry::long_)
        .def_readwrite("altitude", &gsf_nav_entry::altitude)
        .def_readwrite("pos_", &gsf_nav_entry::pos_)
        .def_static("parse_file", &parse_file_from_str<gsf_nav_entry>)
        .def_static("parse_folder", &parse_folder_from_str<gsf_nav_entry>)
        .def_static("read_data", &read_data_from_str<gsf_nav_entry::EntriesT>);

    //m.def("gsf_mbes_ping.parse_file", &parse_file_from_str<gsf_mbes_ping>, "A function which parses gsf mbes pings");
    //m.def("gsf_nav_entry.parse_file", &parse_file_from_str<gsf_nav_entry>, "A function which parses gsf nav entries");
    //m.def("gsf_sound_speed.parse_file", &parse_file_from_str<gsf_sound_speed>, "A function which parses gsf sound speeds");
    //m.def("parse_folder", &parse_folder_from_str<gsf_mbes_ping>, "A function which parses gsf mbes pings from folder");
    //m.def("read_data", &read_data_from_str<gsf_mbes_ping::PingsT>, "A function which reads gsf pings from a .cereal file");
    m.def("write_data", &write_data_from_str<gsf_mbes_ping::PingsT>, "A function which writes gsf pings to a .cereal file");
    m.def("write_data", &write_data_from_str<gsf_nav_entry::EntriesT>, "A function which writes gsf nav entries to a .cereal file");
    m.def("write_data", &write_data_from_str<gsf_sound_speed::SpeedsT>, "A function which writes gsf sound speeds to a .cereal file");
    m.def("convert_pings", &convert_pings, "A function which converts gsf_mbes_ping::EntriesT to mbes_ping::EntriesT");
}
