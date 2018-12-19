#include <iostream>
#include <cereal/archives/json.hpp>
#include <data_tools/xtf_data.h>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace std_data;
using namespace xtf_data;

int main(int argc, char** argv)
{
    boost::filesystem::path folder("/home/nbore/Data/KTH_GBG_PING/Ping_Unprocessed/2-Raw_Data/SSS/xtf");
    xtf_sss_ping::PingsT pings = parse_folder<xtf_sss_ping>(folder);
    
    for (auto pos = pings.begin(); pos != pings.end(); ) {
        auto next = std::find_if(pos, pings.end(), [&](const xtf_sss_ping& ping) {
            return ping.first_in_file_ && (&ping != &(*pos));
        });
        xtf_sss_ping::PingsT track_pings(pos, next);
        cv::Mat waterfall_img = make_waterfall_image(track_pings);
        cv::imshow("My image", waterfall_img);
        cv::waitKey();
        pos = next;
    }

    int counter = 0;
    for (xtf_sss_ping ping : pings) {
        if (counter % 1000 == 0) {
            cereal::JSONOutputArchive ar(std::cout);
            ar(ping);
        }
        ++counter;
    }
    return 0;
}

