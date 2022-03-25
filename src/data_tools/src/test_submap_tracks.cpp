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

#include <iostream>
#include <data_tools/std_data.h>
#include <data_tools/gsf_data.h>
#include <data_tools/colormap.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace std_data;
using namespace gsf_data;

void divide_gsf_map(mbes_ping::PingsT& pings)
{
    pings[3926].first_in_file_ = false;
    pings[5200].first_in_file_ = true;
    pings[10151].first_in_file_ = false;
    pings[10400].first_in_file_ = true;
    pings[15500].first_in_file_ = true;
    pings[16376].first_in_file_ = false;
    pings[20700].first_in_file_ = true;
    pings[22601].first_in_file_ = false;
    pings[25800].first_in_file_ = true;
    pings[28827].first_in_file_ = false;
    pings[30750].first_in_file_ = true;
    pings[33300].first_in_file_ = true;
    pings[34500].first_in_file_ = true;
    pings[35052].first_in_file_ = false;
    pings[36800].first_in_file_ = true;
    pings[37800].first_in_file_ = true;
    pings[40300].first_in_file_ = true;
    pings[43700].first_in_file_ = true;
    pings[44600].first_in_file_ = true;
    pings[47000].first_in_file_ = true;
    pings[47502].first_in_file_ = false;
    pings[48000].first_in_file_ = true;
}

int main(int argc, char** argv)
{
    // read the data
    boost::filesystem::path folder("/home/nbore/Data/ACFR-tas200810pockmarks/PROCESSED_DATA/r20081015_221314_butts_pockmarks_23_overlappinggrids/bpslam20110606/DT20081015_221314_gsf");
    gsf_mbes_ping::PingsT pings_unfiltered = parse_folder<gsf_mbes_ping>(folder);
    std::stable_sort(pings_unfiltered.begin(), pings_unfiltered.end(), [](const gsf_mbes_ping& ping1, const gsf_mbes_ping& ping2) {
        return ping1.time_stamp_ < ping2.time_stamp_;
    });
    gsf_mbes_ping::PingsT pings(pings_unfiltered.begin() + 2300, pings_unfiltered.begin() + 52600);
    pings[0].first_in_file_ = true;

    gsf_nav_entry::EntriesT entries = parse_file<gsf_nav_entry>(boost::filesystem::path("/home/nbore/Data/ACFR-tas200810pockmarks/PROCESSED_DATA/r20081015_221314_butts_pockmarks_23_overlappinggrids/bpslam20110606/dr_pose_est.data"));
    
    gsf_sound_speed::SpeedsT speeds = parse_file<gsf_sound_speed>(boost::filesystem::path("/home/nbore/Data/ACFR-tas200810pockmarks/PROCESSED_DATA/r20081015_221314_butts_pockmarks_23_overlappinggrids/bpslam20110606/DT20081015_221314_gsf/sound_speed.data"));

    match_sound_speeds(pings, speeds);
    mbes_ping::PingsT new_pings = convert_matched_entries(pings, entries);
    divide_gsf_map(new_pings);

    // find the image parameters
    auto xcomp = [](const mbes_ping& p1, const mbes_ping& p2) {
        return p1.pos_[0] < p2.pos_[0];
    };
    auto ycomp = [](const mbes_ping& p1, const mbes_ping& p2) {
        return p1.pos_[1] < p2.pos_[1];
    };
    double maxx = std::max_element(new_pings.begin(), new_pings.end(), xcomp)->pos_[0];
    double minx = std::min_element(new_pings.begin(), new_pings.end(), xcomp)->pos_[0];
    double maxy = std::max_element(new_pings.begin(), new_pings.end(), ycomp)->pos_[1];
    double miny = std::min_element(new_pings.begin(), new_pings.end(), ycomp)->pos_[1];

    cout << "Min X: " << minx << ", Max X: " << maxx << ", Min Y: " << miny << ", Max Y: " << maxy << endl;

    int rows = 1000;
    int cols = 1000;

    double xres = double(cols)/(maxx - minx);
    double yres = double(rows)/(maxy - miny);

    double res = std::min(xres, yres);

    double x0 = .5*(double(cols) - res*(maxx-minx));
    double y0 = .5*(double(rows) - res*(maxy-miny));

    cout << xres << ", " << yres << endl;
    cv::Mat track_img = cv::Mat(rows, cols, CV_8UC3, cv::Scalar(255, 255, 255));

    vector<vector<cv::Point2f> > curve_points;
    int i = 0;
    int counter = 0;
    for (const mbes_ping& ping : new_pings) {
        cv::Point2f pt(x0+res*(ping.pos_[0]-minx), rows-y0-res*(ping.pos_[1]-miny)-1);
        if (ping.first_in_file_) {
            curve_points.push_back(vector<cv::Point2f>());
            cv::Point org(pt.x, pt.y);
            cv::putText(track_img, std::to_string(counter) + ":" + std::to_string(i), org, cv::FONT_HERSHEY_PLAIN, 1., cv::Scalar(0, 0, 255), 1, 8, false);
            ++counter;
        }
        if (i % 500 == 0) {
            cv::Point org(pt.x, pt.y);
            cv::putText(track_img, std::to_string(i), org, cv::FONT_HERSHEY_PLAIN, 1., cv::Scalar(0, 0, 0), 1, 8, false);
        }
        curve_points.back().push_back(pt);
        ++i;
    }

    i = 0;
    for (vector<cv::Point2f>& cp : curve_points) {
        cv::Scalar color(colormap[i%43][2], colormap[i%43][1], colormap[i%43][0]);
        cv::Mat curve(cp, true);
        curve.convertTo(curve, CV_32S); //adapt type for polylines
        cv::polylines(track_img, curve, false, color, 2, cv::LINE_AA);
        ++i;
    }

    cv::imshow("Track", track_img);
    cv::waitKey();

    return 0;
}

