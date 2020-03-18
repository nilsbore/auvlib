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

#include <bathy_maps/sss_gen_sim.h>
#include <bathy_maps/base_draper.h>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

using namespace std_data;
using namespace csv_data;

namespace py = pybind11;

PYBIND11_MODULE(sss_gen_sim, m) {
    m.doc() = "Functions for simulating sidescan data from mesh"; // optional module docstring

    py::class_<SSSGenSim>(m, "SSSGenSim", "Class for simulating sidescan pings from a bathymetry mesh")
        // Methods inherited from BaseDraper:
        .def(py::init<const Eigen::MatrixXd&, const Eigen::MatrixXi&,
                      const std_data::sss_ping::PingsT&, const SSSGenSim::BoundsT&,
                      const csv_asvp_sound_speed::EntriesT&, const Eigen::MatrixXd&>())
        .def("set_sidescan_yaw", &SSSGenSim::set_sidescan_yaw, "Set yaw correction of sidescan with respect to nav frame")
        .def("set_sidescan_port_stbd_offsets", &SSSGenSim::set_sidescan_port_stbd_offsets, "Set offsets of sidescan port and stbd sides with respect to nav frame")
        .def("set_ray_tracing_enabled", &SSSGenSim::set_ray_tracing_enabled, "Set if ray tracing through water layers should be enabled. Takes more time but is recommended if there are large speed differences")
        .def("set_vehicle_mesh", &SSSGenSim::set_vehicle_mesh, "Provide the viewer with a vehicle model, purely for visualization")
        .def("show", &SSSGenSim::show, "Start the draping, and show the visualizer")
        .def("draw_sim_waterfall", &SSSGenSim::draw_sim_waterfall, "Get the simulated waterfall from the incidence image")
        .def("draw_model_waterfall", &SSSGenSim::draw_model_waterfall, "Get the idealized model waterfall from the incidence image")
        // Methods unique to SSSGenSim:
        .def("set_gen_callback", &SSSGenSim::set_gen_callback, "Set the function that generates sidescan patches from bathymetry")
        .def("set_gen_window_height", &SSSGenSim::set_gen_window_height, "Set the height of the window that we are sending to the network")
        .def("set_sss_from_bathy", &SSSGenSim::set_sss_from_bathy, "If true, predicts sidescan from local bathymetry window")
        .def("set_sss_from_waterfall", &SSSGenSim::set_sss_from_waterfall, "If true, predicts sidescan from waterfall elevation image");

}
