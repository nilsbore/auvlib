#ifndef BENCHMARK_H
#define BENCHMARK_H

#include <data_tools/data_structures.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace benchmark {

struct track_error_benchmark {

    // the name of the dataset that we benchmark
    std::string dataset_name;

    // this contains all added tracks assembled into one image
    std::string track_img_path;
    // this contains all added track error images
    std::map<std::string, std::string> error_img_paths;

    // TODO: this should just be the normal pings instead
    //std::vector<std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > > submap_tracks;

    // NOTE: this is the potentially distorted input data
    data_structures::mbes_ping::PingsT input_pings;

    // NOTE: this is needed to compare with the ground truth
    data_structures::pt_submaps::TransT gt_track;
    //std::map<std::string, pt_submaps::TransT> tracks;
    std::map<std::string, double> track_rms_errors;
    std::map<std::string, double> consistency_rms_errors;
    double min_consistency_error;
    double max_consistency_error;

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
    void add_ground_truth(data_structures::mbes_ping::PingsT& pings);
    void add_initial(data_structures::mbes_ping::PingsT& pings);
    void add_benchmark(data_structures::mbes_ping::PingsT& pings, const std::string& name);
    void add_benchmark(data_structures::pt_submaps::TransT& trans_corr, data_structures::pt_submaps::RotsT& rots_corr, const std::string& name);
    void print_summary();

    void track_img_params(data_structures::mbes_ping::PingsT& pings, int rows=1000, int cols=1000);
    void draw_track_img(data_structures::mbes_ping::PingsT& pings, cv::Mat& img, const cv::Scalar& color, const std::string& name);
    //void draw_track_img(pt_submaps::TransT& positions);
    void draw_track_legend();
    double compute_rms_error(data_structures::mbes_ping::PingsT& pings);
    std::pair<double, cv::Mat> compute_draw_consistency_map(data_structures::mbes_ping::PingsT& pings);
    std::pair<double, cv::Mat> compute_draw_error_consistency_map(data_structures::mbes_ping::PingsT& pings);
    cv::Mat draw_height_map(data_structures::mbes_ping::PingsT& pings);

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
    data_structures::pt_submaps::MatchesT registration_pairs;

    static data_structures::mbes_ping::PingsT get_submap_pings_pair(const data_structures::mbes_ping::PingsT& pings, int i, int j);
    static data_structures::mbes_ping::PingsT get_submap_pings_index(const data_structures::mbes_ping::PingsT& pings, int i);
    void add_registration_benchmark(data_structures::mbes_ping::PingsT& initial_pings, data_structures::mbes_ping::PingsT& optimized_pings, int i, int j);
    void add_registration_benchmark(data_structures::mbes_ping::PingsT& initial_pings, data_structures::pt_submaps::TransT& trans_corr, data_structures::pt_submaps::RotsT& rots_corr, int i, int j);
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
