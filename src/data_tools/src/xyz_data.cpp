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

xyz_data::Points transform_points(const Eigen::Matrix4d& T, xyz_data::Points& points)
{
    xyz_data::Points transformed_points = points;
    Eigen::Affine3d transform(T);
    for (Eigen::Vector3d& p : transformed_points) {
        p = transform*p;
    }
    return transformed_points;
}

xyz_data::Points subsample_points(const xyz_data::Points& points, int skip)
{
    xyz_data::Points subsampled_points;
    for (int i = 0; i < points.size(); i += skip) {
        subsampled_points.push_back(points[i]);
    }
    return subsampled_points;
}

vector<xyz_data::Points> from_pings(const std_data::mbes_ping::PingsT& pings)
{
    vector<xyz_data::Points> maps;

    for (auto pos = pings.begin(); pos != pings.end(); ) {
        auto next = std::find_if(pos, pings.end(), [&](const std_data::mbes_ping& ping) {
            return ping.first_in_file_ && (&ping != &(*pos));
        });

        xyz_data::Points map;
        for (auto iter = pos; iter != next; ++iter) {
            map.insert(map.end(), iter->beams.begin(), iter->beams.end());
        }

        if (!map.empty()) {
            maps.push_back(map);
        }

        pos = next;
    }

    return maps;
}

xyz_data::Points from_matrix(const Eigen::MatrixXd& P)
{
    xyz_data::Points points;
    points.reserve(P.rows());
    for (int i = 0; i < P.rows(); ++i) {
        points.push_back(P.block<1, 3>(i, 0).transpose());
    }
    return points;
}

Eigen::MatrixXd to_matrix(const Points& points)
{
    Eigen::MatrixXd P(points.size(), 3);
    int counter = 0;
    for (const Eigen::Vector3d& p : points) {
        P.row(counter) = p.transpose();
        ++counter;
    }
    return P;
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

    cout << "Using separator: " << separator << endl;

    //size_t i = 0;
    size_t counter = 0;
    Eigen::Vector3d p;
    while (std::getline(infile, line))  // this does the checking!
    {
        /*
        if (i < 10) {
            ++i;
            continue;
        }
        else {
            i = 0;
        }
        */
        if (line.empty() || line[0] == '\n') {// || !isdigit(line[0])) {
            continue;
        }
        istringstream iss(line);
		if (separator == ' ') {
            iss >> p(0) >> p(1) >> p(2);
        }
        else {
            //std::getline(iss, p(0), separator);
            //std::getline(iss, p(1), separator);
            //std::getline(iss, p(2), separator);
            for (int j = 0; j < 3; ++j) {
                std::getline(iss, line, separator);
                p[j] = std::stod(line.c_str());
            }
        }
        if (counter >= cloud.capacity()) {
            cout << "Reserving space: " << cloud.size() + 1000000 << endl;
            cloud.reserve(cloud.size() + 1000000);
        }
        if (counter % 1000 == 0) {
            cout << "Point: " << p.transpose() << endl;
        }
        cloud.push_back(p);
        ++counter;
    }

    return cloud;
}

} // namespace std_data
