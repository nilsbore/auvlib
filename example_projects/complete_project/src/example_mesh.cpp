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

#include <cereal/archives/json.hpp>

#include <cxxopts.hpp>
#include <data_tools/gsf_data.h>
#include <data_tools/csv_data.h>
#include <data_tools/xtf_data.h>
#include <data_tools/transforms.h>
#include <data_tools/submaps.h>

#include <bathy_maps/draw_map.h>
#include <bathy_maps/mesh_map.h>
#include <bathy_maps/patch_draper.h>

#include <opencv2/highgui/highgui.hpp>
#include <chrono>

using namespace std;
using namespace std_data;
using namespace csv_data;
using namespace gsf_data;
using namespace xtf_data;

// we should probably put this in data_tools eventually, it's very convenient
template <typename T>
typename T::PingsT load_or_parse_pings(const boost::filesystem::path& swaths_folder, const string& dataset_name)
{
    typename T::PingsT pings;
    if (boost::filesystem::exists("ping_swaths_" + dataset_name + ".cereal")) {
        cout << "Reading saved pings..." << endl;
        std::ifstream is("ping_swaths_" + dataset_name + ".cereal", std::ifstream::binary);
        {
			cereal::BinaryInputArchive archive(is);
			archive(pings);
        }
        is.close();
    }
    else {
        cout << "Parsing pings..." << endl;
        pings = parse_folder<T>(swaths_folder);
        std::stable_sort(pings.begin(), pings.end(), [](const T& ping1, const T& ping2) {
            return ping1.time_stamp_ < ping2.time_stamp_;
        });
        std::ofstream os("ping_swaths_" + dataset_name + ".cereal", std::ofstream::binary);
        {
            cereal::BinaryOutputArchive archive(os);
			archive(pings);
        }
        os.close();
    }
    return pings;
}

csv_nav_entry::EntriesT load_or_parse_entries(const boost::filesystem::path& poses_path)
{
    csv_nav_entry::EntriesT entries;
    if (boost::filesystem::exists("ping_poses.cereal")) {
        cout << "Reading saved poses..." << endl;
        std::ifstream is("ping_poses.cereal", std::ifstream::binary);
        {
			cereal::BinaryInputArchive archive(is);
			archive(entries);
        }
        is.close();
    }
    else {
        cout << "Parsing poses..." << endl;
        entries = parse_file<csv_nav_entry>(poses_path);
        std::ofstream os("ping_poses.cereal", std::ofstream::binary);
        {
            cereal::BinaryOutputArchive archive(os);
			archive(entries);
        }
        os.close();
    }
    return entries;
}

int main(int argc, char** argv)
{
    string mbes_folder_str;
    string sss_folder_str;
    string entries_file_str;
    string file_str;
	double lsq = 7; //10.;
	double sigma = 1.; //5.;
	double s0 = 0.03; //.2;
    double pose_sigma = 0.2; //0.4;
    double sensor_yaw = 5.*M_PI/180.;
    string dataset_name = "ping_mesh";

	cxxopts::Options options("MyProgram", "One line description of MyProgram");
	options.add_options()
      ("help", "Print help")
      ("swaths", "Input gsf mb swaths folder pre deployment", cxxopts::value(mbes_folder_str))
      ("sss", "Input gsf mb swaths folder pre deployment", cxxopts::value(sss_folder_str))
      ("nav", "Nav entries file", cxxopts::value(entries_file_str))
      ("file", "Output file", cxxopts::value(file_str))
      ("lsq", "RBF length scale", cxxopts::value(lsq))
      ("sigma", "RBF scale", cxxopts::value(sigma))
      ("pose_sigma", "The standard deviation pose update per meter", cxxopts::value(pose_sigma))
      ("name", "The name of the dataset, without spaces", cxxopts::value(dataset_name))
      ("s0", "Measurement noise", cxxopts::value(s0));

    auto result = options.parse(argc, argv);
	if (result.count("help")) {
        cout << options.help({"", "Group"}) << endl;
        exit(0);
	}
    if (result.count("swaths") == 0 || result.count("sss") == 0) {
		cout << "Please provide input swaths and sss args..." << endl;
		exit(0);
    }
    if (result.count("file") == 0) {
		cout << "Please provide output file arg..." << endl;
		exit(0);
    }
	
	boost::filesystem::path mbes_folder(mbes_folder_str);
	boost::filesystem::path sss_folder(sss_folder_str);
	boost::filesystem::path entries_file(entries_file_str);
    boost::filesystem::path path(file_str);

	cout << "Input mbes folder : " << mbes_folder << endl;
	cout << "Input sss folder : " << sss_folder << endl;
	cout << "Output file : " << path << endl;

    csv_nav_entry::EntriesT entries = load_or_parse_entries(entries_file);
    for (int counter = 0; counter < entries.size(); counter += 10000) {
        cout << "Time stamp: " << entries[counter].time_string_ << endl;
    }

    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    mesh_map::BoundsT bounds;

    // we need to separate the reading of mbes and side scan pings since they consume a lot of memory
    {
        gsf_mbes_ping::PingsT pings_mbes = load_or_parse_pings<gsf_mbes_ping>(mbes_folder, dataset_name + "_mbes");
        mbes_ping::PingsT pings = convert_pings(pings_mbes);

        tie(V, F, bounds) = mesh_map::mesh_from_pings(pings);
    }
    mesh_map::show_mesh(V, F);
    cout << "SSS" << endl;

    {
        xtf_sss_ping::PingsT pings_sss = load_or_parse_pings<xtf_sss_ping>(sss_folder, dataset_name + "_sss");
        for (int counter = 0; counter < pings_sss.size(); counter += 100) {
            cout << "Time stamp: " << pings_sss[counter].time_string_ << endl;
        }
        pings_sss = convert_matched_entries(pings_sss, entries);
        pings_sss = correct_sensor_offset(pings_sss, Eigen::Vector3d(2., -1.5, 0.));
        cout << "Number of entries in file " << entries_file << ": " << entries.size() << endl;
        /*for (xtf_sss_ping& ping : pings_sss) {
            ping.pitch_ = 0.2;
            //cereal::JSONOutputArchive ar(std::cout);
            //ar(ping);
        }*/
        cv::Mat waterfall_img = make_waterfall_image(pings_sss);
        cv::imshow("My image", waterfall_img);
        cv::waitKey();
        csv_asvp_sound_speed::EntriesT sound_speeds;
        drape_patches(V, F, bounds, pings_sss, sound_speeds, sensor_yaw);
    }

    return 0;
}

