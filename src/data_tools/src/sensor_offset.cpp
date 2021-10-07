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
    map<string, pos_offset> parse_offset_file(const string& file) {

        ifstream input;
        input.open(file, ifstream::in);

        if (input.fail()) {
            cout << "ERROR: Cannot open the file..." << endl;
            exit(0);
        }

        string line;
        map<string, pos_offset> offset_map;
        // regex variables for string matching
        regex sensor_name("\\[\\w+\\]");
        cmatch match;
        while (getline(input, line)) {
            boost::algorithm::trim(line);
            if (regex_match(line.c_str(), match, sensor_name)) {
                offset_map[line] = parse_offset_for_one_sensor(input);
                cout << "matched sensor name: " << match.str() << "\n";
            }
        }
        input.close();

        return offset_map;
    }

    //TODO: finish parsing the file
    pos_offset parse_offset_for_one_sensor(ifstream& input) {
        pos_offset offset = {1., 2., 3.};
        return offset;
    }
}
