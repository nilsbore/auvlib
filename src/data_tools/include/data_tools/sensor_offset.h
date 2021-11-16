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

#ifndef SENSOR_OFFSET_H
#define SENSOR_OFFSET_H

#include <Eigen/Dense>
#include <map>
#include <string>
#include <regex>

namespace sensor_offset {
    struct SonarOffset {
        Eigen::Vector3d mbes_tx; //Multibeam transmitter head
        Eigen::Vector3d mbes_rx; //Multibeam receiver head
        Eigen::Vector3d sss_port; //Sidescan port offset
        Eigen::Vector3d sss_stbd; //Sidescan starboard offset
    };

    // Regex expressions for parsing sensor offset file
    static std::regex regex_sensor_name("\\[(\.+)\\]");
    static std::regex regex_xpos("x = ([-]?\\d*\\.\\d*)");
    static std::regex regex_ypos("y = ([-]?\\d*\\.\\d*)");
    static std::regex regex_zpos("z = ([-]?\\d*\\.\\d*)");

    // default sensor arguments are those of RAN
    SonarOffset get_sonar_offset_from_file(const std::string& file,
            const std::string& mbes_tx_name = "EMSonarHeadTX",
            const std::string& mbes_rx_name ="EMSonarHeadRX",
            const std::string& sss_port_name = "Edge tech-SSS-Port",
            const std::string& sss_stbd_name = "Edge tech-SSS-STB");
    std::map<std::string, Eigen::Vector3d> parse_offset_file(const std::string& file);
    Eigen::Vector3d parse_offset_for_one_sensor(std::ifstream& input);
}

#endif
