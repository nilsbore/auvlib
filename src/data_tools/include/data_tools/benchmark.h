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

#ifndef BENCHMARK_H
#define BENCHMARK_H

#include <data_tools/std_data.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace benchmark {

struct track_error_benchmark {

    using PointsT = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd> >;

    // the name of the dataset that we benchmark
    std::string dataset_name;

    // this contains all added tracks assembled into one image
    std::string track_img_path;
    // this contains all added track error images
    std::map<std::string, std::string> error_img_paths;

    // TODO: this should just be the normal pings instead
    //std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > > submap_tracks;

    // NOTE: this is the potentially distorted input data
    std_data::mbes_ping::PingsT input_pings;

    // NOTE: this is needed to compare with the ground truth
    std_data::pt_submaps::TransT gt_track;
    //std::map<std::string, pt_submaps::TransT> tracks;
    std::map<std::string, double> track_rms_errors;
    std::map<std::string, double> consistency_rms_errors;
    double min_consistency_error;
    double max_consistency_error;
    double min_depth_;
    double max_depth_;

    // TODO: get this from on of the dicts instead
    //int nbr_tracks_drawn;

    std::array<double, 5> params;
    Eigen::Vector3d submap_origin;

    // NOTE: this is here for convenience
    cv::Mat track_img;

    track_error_benchmark() : dataset_name("default")
    {
        min_consistency_error = -1.;
        max_consistency_error = -1.;
    }

    track_error_benchmark(const std::string& dataset_name) : dataset_name(dataset_name)
    {
        min_consistency_error = -1.;
        max_consistency_error = -1.;
    }

    // these 5 functions should be the main way of interfacing with this class
    void add_ground_truth(std_data::mbes_ping::PingsT& pings);
    void add_initial(std_data::mbes_ping::PingsT& pings);
    void add_benchmark(std_data::mbes_ping::PingsT& pings, const std::string& name);
    void add_benchmark(std_data::pt_submaps::TransT& trans_corr, std_data::pt_submaps::RotsT& rots_corr, const std::string& name);
    void print_summary();

    // Overloaded functions to work with input submaps in PointsT format
    void add_ground_truth(PointsT &map_points, PointsT &track_points);
    void add_benchmark(PointsT &maps_points, PointsT &tracks_points, const std::string &name);

    void track_img_params(PointsT& points_maps, int rows=1000, int cols=1000);
    cv::Mat draw_height_map(PointsT &points_maps);
    std::vector<std::vector<std::vector<Eigen::MatrixXd> > > create_grids_from_pings(std_data::mbes_ping::PingsT& pings);
    std::vector<std::vector<std::vector<Eigen::MatrixXd> > > create_grids_from_matrices(PointsT& points_maps);
    std::pair<double, Eigen::MatrixXd> compute_consistency_error(
            std::vector<std::vector<std::vector<Eigen::MatrixXd> > >& grid_maps);
    cv::Mat draw_error_consistency_map(Eigen::MatrixXd values);

    // Draw heightmap of submaps
    cv::Mat draw_height_submap(PointsT &map_points, PointsT &track_points, const int &submap_number);
    void map_draw_params(PointsT& map_points, PointsT& track_points, const int& submap_number);

    void track_img_params(std_data::mbes_ping::PingsT& pings, int rows=1000, int cols=1000);
    void draw_track_img(std_data::mbes_ping::PingsT& pings, cv::Mat& img, const cv::Scalar& color, const std::string& name);
    //void draw_track_img(pt_submaps::TransT& positions);
    void draw_track_legend();
    double compute_rms_error(std_data::mbes_ping::PingsT& pings);
    std::pair<double, cv::Mat> compute_draw_consistency_map(std_data::mbes_ping::PingsT& pings);

    cv::Mat draw_height_map(std_data::mbes_ping::PingsT& pings);

    template <class Archive>
    void save(Archive& ar) const
    {
        if (track_img.rows > 0) {
            cv::imwrite(track_img_path, track_img);
        }
        ar(dataset_name, track_img_path, error_img_paths, input_pings, gt_track, track_rms_errors,
           consistency_rms_errors, min_consistency_error, max_consistency_error, params, submap_origin);
    }

    template <class Archive>
    void load(Archive& ar)
    {
        ar(dataset_name, track_img_path, error_img_paths, input_pings, gt_track, track_rms_errors,
           consistency_rms_errors, min_consistency_error, max_consistency_error, params, submap_origin);
        if (!track_img_path.empty()) {
            track_img = cv::imread(track_img_path);
        }
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

struct registration_summary_benchmark {

    std::string dataset_name;
    std::vector<track_error_benchmark> benchmarks;
    std_data::pt_submaps::MatchesT registration_pairs;

    static std_data::mbes_ping::PingsT get_submap_pings_pair(const std_data::mbes_ping::PingsT& pings, int i, int j);
    static std_data::mbes_ping::PingsT get_submap_pings_index(const std_data::mbes_ping::PingsT& pings, int i);
    void add_registration_benchmark(std_data::mbes_ping::PingsT& initial_pings, std_data::mbes_ping::PingsT& optimized_pings, int i, int j);
    void add_registration_benchmark(std_data::mbes_ping::PingsT& initial_pings, std_data::pt_submaps::TransT& trans_corr, std_data::pt_submaps::RotsT& rots_corr, int i, int j);
    void print_summary();

    registration_summary_benchmark(const std::string& dataset_name) : dataset_name(dataset_name)
    {

    }

    template <class Archive>
    void serialize( Archive & ar )
    {
        ar(dataset_name, benchmarks, registration_pairs);
    }

};

} // namespace benchmark

#endif // BENCHMARK_H
