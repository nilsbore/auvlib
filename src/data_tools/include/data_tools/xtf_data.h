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

#ifndef XTF_DATA_H
#define XTF_DATA_H

#include <data_tools/navi_data.h>
#include <Eigen/Dense>
#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS
#include <opencv2/core/core.hpp>

namespace xtf_data {

// the xtf pings and std pings are the same
using xtf_sss_ping_side = std_data::sss_ping_side;
using xtf_sss_ping = std_data::sss_ping;

/*
struct xtf_sss_ping_side : public std_data::sss_ping_side {

	template <class Archive>
    void serialize( Archive & ar )
    {
        ar(cereal::base_class<std_data::sss_ping_side>( this ));
    }

};
*/

/*
struct xtf_sss_ping : public std_data::sss_ping {

    //using PingsT = std::vector<xtf_sss_ping, Eigen::aligned_allocator<xtf_sss_ping> >;

	template <class Archive>
    void serialize( Archive & ar )
    {
        ar(cereal::base_class<std_data::sss_ping>( this ));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
*/

cv::Mat make_waterfall_image(const xtf_sss_ping::PingsT& pings);
Eigen::MatrixXd make_eigen_waterfall_image(const xtf_sss_ping::PingsT& pings);
void show_waterfall_image(const xtf_sss_ping::PingsT& pings);

xtf_sss_ping::PingsT correct_sensor_offset(const xtf_sss_ping::PingsT& pings, const Eigen::Vector3d& sensor_offset);
xtf_sss_ping::PingsT match_attitudes(const xtf_sss_ping::PingsT& pings, const std_data::attitude_entry::EntriesT& entries);

} // namespace xtf_data

namespace std_data {

template <>
xtf_data::xtf_sss_ping::PingsT parse_file(const boost::filesystem::path& file);

}

#endif // XTF_DATA_H
