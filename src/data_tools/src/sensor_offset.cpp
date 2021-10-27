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
#include <iostream>
#include <fstream>
#include <string>
#include <regex>
#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/algorithm/string/trim.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS
#include <typeinfo>

using namespace std;

namespace sensor_offset {
    void set_sensor_offset_variables_from_file(const std::string& file, const std::string& mbes_tx_name,
            const std::string& mbes_rx_name, const std::string& sss_port_name, const std::string& sss_stbd_name) {
        auto offset_map = parse_offset_file(file);
        MBES_TX = offset_map[mbes_tx_name];
        MBES_RX = offset_map[mbes_rx_name];
        SSS_PORT = offset_map[sss_port_name];
        SSS_STBD = offset_map[sss_stbd_name];

        cout << "New sensor offset: " << endl;
        cout << "MBES_TX: (" << MBES_TX[0] << ", " << MBES_TX[1] << ", " << MBES_TX[2] << ")" << endl;
        cout << "MBES_RX: (" << MBES_RX[0] << ", " << MBES_RX[1] << ", " << MBES_RX[2] << ")" << endl;
        cout << "SSS_PORT: (" << SSS_PORT[0] << ", " << SSS_PORT[1] << ", " << SSS_PORT[2] << ")" << endl;
        cout << "SSS_STBD: (" << SSS_STBD[0] << ", " << SSS_STBD[1] << ", " << SSS_STBD[2] << ")" << endl;
    }

    map<string, Eigen::Vector3d> parse_offset_file(const string& file) {

        ifstream input;
        input.open(file, ifstream::in);

        if (input.fail()) {
            cout << "ERROR: Cannot open the file..." << endl;
            exit(0);
        }

        string line;
        map<string, Eigen::Vector3d> offset_map;
        // regex variables for string matching
        cmatch match;
        while (getline(input, line)) {
            boost::algorithm::trim(line);
            if (regex_match(line.c_str(), match, regex_sensor_name)) {
                auto sensor_name = match[1].str();
                // ignore the [Version] tag
                if (sensor_name == "Version") {
                    continue;
                }
                offset_map[sensor_name] = parse_offset_for_one_sensor(input);
            }
        }
        input.close();

        return offset_map;
    }

    Eigen::Vector3d parse_offset_for_one_sensor(ifstream& input) {
        string line;
        cmatch match;
        Eigen::Vector3d offset;

        while (getline(input, line)) {
            boost::algorithm::trim(line);
            auto line_c_str = line.c_str();
            if (regex_match(line_c_str, match, regex_xpos)) {
                offset[0] = stof(match[1]);
            } else if (regex_match(line_c_str, match, regex_ypos)) {
                offset[1] = stof(match[1]);
            } else if (regex_match(line_c_str, match, regex_zpos)) {
                offset[2] = stof(match[1]);
            }

            // Return if the next token is [ (indicates a new sensor)
            auto next = input.peek();
            if (next == '[') {
                break;
            }
        }
        return offset;
    }
}
