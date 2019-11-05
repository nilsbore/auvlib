/* Copyright 2018 Nils Bore (nbore@kth.se), Yiping Xie (yipingx@kth.se)
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef JSF_DATA_H
#define JSF_DATA_H

#include <data_tools/std_data.h>
#include <libjsf/jsf.h>
#include <Eigen/Dense>
#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS
#include <opencv2/core/core.hpp>


namespace jsf_data {

struct jsf_sss_ping_side
{
    std::vector<float> pings;
    std::vector<float> pings_phase;
	template <class Archive>
    void serialize( Archive & ar )
    {
        ar(CEREAL_NVP(pings), CEREAL_NVP(pings_phase) );
    }
};

struct jsf_sss_ping
{
    using PingsT = std::vector<jsf_sss_ping, Eigen::aligned_allocator<jsf_sss_ping> >;

    bool first_in_file_;
    std::string time_string_; // readable time stamp string
    long long time_stamp_; // posix time stamp
    jsf_sss_ping_side port;
    jsf_sss_ping_side stbd;
    bool is_bad;


	template <class Archive>
    void serialize( Archive & ar )
    {
        ar(CEREAL_NVP(first_in_file_), CEREAL_NVP(time_string_), CEREAL_NVP(time_stamp_), CEREAL_NVP(port), CEREAL_NVP(stbd), CEREAL_NVP(is_bad));
    }
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


cv::Mat make_waterfall_image(const jsf_sss_ping::PingsT& pings);
void show_waterfall_image(const jsf_sss_ping::PingsT& pings);

} // namespace jsf_data

namespace std_data {

template <>
jsf_data::jsf_sss_ping::PingsT parse_file(const boost::filesystem::path& file);

}

#endif // JSF_DATA_H
