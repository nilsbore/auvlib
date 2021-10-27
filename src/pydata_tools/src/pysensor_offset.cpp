/* Copyright 2021 Li Ling (liling@kth.se)
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <data_tools/sensor_offset.h>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

using namespace sensor_offset;

namespace py = pybind11;

PYBIND11_MODULE(sensor_offset, m) {
    m.doc() = "Module for parsing sensor offset files, e.g. arms.init file from Kongsberg Hugin AUV";

    // Global sensor offset variables
    m.attr("MBES_RX") = &MBES_RX; //multibeam receiver head offset
    m.attr("MBES_TX") = &MBES_TX; //multibeam transmitter head offset
    m.attr("SSS_PORT") = &SSS_PORT; //sidescan port side offset
    m.attr("SSS_STBD") = &SSS_STBD; //sidescan starboard side offset

    m.def("parse_offset_file", &parse_offset_file, "Parse sensor offset file. Note that the following sign convention is assumed: +x=forward, +y=starboard, +z=down");
    m.def("set_sensor_offset_variables_from_file", &set_sensor_offset_variables_from_file,
            py::arg("file"),
            py::arg("mbes_tx_name")="EMSonarHeadTX",
            py::arg("mbes_rx_name")="EMSonarHeadRX",
            py::arg("sss_port_name")="Edge tech-SSS-Port",
            py::arg("sss_stbd_name")="Edge tech-SSS-STB",
            "Set MBES_RX, MBES_TX, SSS_PORT, SSS_STBD using the provided file and sensor names. Default sensor names are those of RAN");
}
