#ifndef XTF_DATA_H
#define XTF_DATA_H

#include <data_tools/navi_data.h>
#include <Eigen/Dense>
#include <boost/filesystem.hpp>
#include <opencv2/core/core.hpp>

struct xtf_sss_ping
{
    using PingsT = std::vector<xtf_sss_ping, Eigen::aligned_allocator<xtf_sss_ping> >;

    std::vector<int> port_pings;
    std::vector<int> stbd_pings;
    bool first_in_file_;

	template <class Archive>
    void serialize( Archive & ar )
    {
        ar(CEREAL_NVP(port_pings), CEREAL_NVP(stbd_pings), CEREAL_NVP(first_in_file_));
    }

};

template <>
xtf_sss_ping::PingsT parse_file(const boost::filesystem::path& file);

cv::Mat make_waterfall_image(const xtf_sss_ping::PingsT& pings);

#endif // XTF_DATA_H
