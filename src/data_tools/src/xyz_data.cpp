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

#include <data_tools/xyz_data.h>
#include <data_tools/lat_long_utm.h>

using namespace std;

namespace xyz_data {

xyz_data::Points subsample_cloud(const xyz_data::Points& cloud)
{
    return cloud;
}

} // namespace xyz_data

namespace std_data {

template <>
xyz_data::Points parse_file(const boost::filesystem::path& file)
{
    xyz_data::Points cloud;
    cloud.reserve(1000000);
    string line;
    std::ifstream infile(file.string());
    if (!std::getline(infile, line)) {
        cout << "File " << file << " did not contain any information!" << endl;
        return cloud;
    }
    char separator = ' ';
    if (line.find(',') != string::npos) {
        separator = ',';
    }
    //size_t i = 0;
    size_t counter = 0;
    Eigen::Vector3d p;
    while (std::getline(infile, line))  // this does the checking!
    {
        /*if (i < 50) {
            ++i;
            continue;
        }
        else {
            i = 0;
        }*/
        if (line.empty() || line[0] == '\n') {// || !isdigit(line[0])) {
            continue;
        }
        istringstream iss(line);
		//iss >> p(0) >> p(1) >> p(2);
        //std::getline(iss, p(0), separator);
        //std::getline(iss, p(1), separator);
        //std::getline(iss, p(2), separator);
        for (int j = 0; j < 3; ++j) {
            std::getline(iss, line, separator);
            p[j] = std::stod(line.c_str());
        }
        if (counter >= cloud.size()) {
            cloud.reserve(cloud.size() + 1000000);
        }
        //cout << "Point: " << p.transpose() << endl;
        cloud.push_back(p);
        ++counter;
    }

    return cloud;
}

} // namespace std_data
