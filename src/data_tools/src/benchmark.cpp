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

#include <data_tools/benchmark.h>
#include <data_tools/colormap.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <numeric>

using namespace std;

namespace benchmark {

using namespace std_data;

void track_error_benchmark::write_matrix_to_file(const Eigen::MatrixXd& matrix, const std::string& filename){
    Eigen::IOFormat CSVFmt(Eigen::FullPrecision, Eigen::DontAlignCols,
                           ", ", "\n");
    ofstream file((filename + ".csv").c_str());
    file << matrix.format(CSVFmt);
}

void track_error_benchmark::compute_benchmark_range_from_pings(const mbes_ping::PingsT& pings, benchmark_range& range) {
    auto xcomp = [](const mbes_ping& p1, const mbes_ping& p2) {
        return p1.pos_[0] < p2.pos_[0];
    };
    auto ycomp = [](const mbes_ping& p1, const mbes_ping& p2) {
        return p1.pos_[1] < p2.pos_[1];
    };
    range.maxx = std::max_element(pings.begin(), pings.end(), xcomp)->pos_[0];
    range.minx = std::min_element(pings.begin(), pings.end(), xcomp)->pos_[0];
    range.maxy = std::max_element(pings.begin(), pings.end(), ycomp)->pos_[1];
    range.miny = std::min_element(pings.begin(), pings.end(), ycomp)->pos_[1];
}

void track_error_benchmark::compute_benchmark_range_from_gt_track(benchmark_range& range) {
    auto xcomp = [](const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
        return p1[0] < p2[0];
    };
    auto ycomp = [](const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
        return p1[1] < p2[1];
    };

    range.maxx = std::max_element(gt_track.begin(), gt_track.end(), xcomp)->data()[0]+20;
    range.minx = std::min_element(gt_track.begin(), gt_track.end(), xcomp)->data()[0]-20;
    range.maxy = std::max_element(gt_track.begin(), gt_track.end(), ycomp)->data()[1]+20;
    range.miny = std::min_element(gt_track.begin(), gt_track.end(), ycomp)->data()[1]-20;
}

void track_error_benchmark::compute_benchmark_range_from_pointsT(const PointsT& points_maps, benchmark_range& range) {
    double minx = std::numeric_limits<double>::max();
    double miny = std::numeric_limits<double>::max();
    double maxx = std::numeric_limits<double>::lowest();
    double maxy = std::numeric_limits<double>::lowest();

    for (auto point_cloud : points_maps) {
        auto min_coeff = point_cloud.colwise().minCoeff();
        auto max_coeff = point_cloud.colwise().maxCoeff();

        minx = std::min(minx, min_coeff[0]);
        miny = std::min(miny, min_coeff[1]);

        maxx = std::max(maxx, max_coeff[0]);
        maxy = std::max(maxy, max_coeff[1]);

    }
    range.minx = minx;
    range.miny = miny;
    range.maxx = maxx;
    range.maxy = maxy;
}

void track_error_benchmark::compute_params_from_benchmark_range(const benchmark_range& range) {
    cout << "Min X: " << range.minx << ", Max X: " << range.maxx << ", Min Y: " << range.miny << ", Max Y: " << range.maxy << endl;

    double xres = double(benchmark_nbr_cols)/(range.maxx - range.minx);
    double yres = double(benchmark_nbr_rows)/(range.maxy - range.miny);

    double res = std::min(xres, yres);

    double x0 = .5*(double(benchmark_nbr_cols) - res*(range.maxx-range.minx));
    double y0 = .5*(double(benchmark_nbr_rows) - res*(range.maxy-range.miny));

    cout << xres << ", " << yres << endl;

    params[0] = res;
    params[1] = range.minx;
    params[2] = range.miny;
    params[3] = x0;
    params[4] = y0;
}


// res, xmin, ymin, imxmin, imymin
void track_error_benchmark::track_img_params(mbes_ping::PingsT& pings)
{
    benchmark_range range;
    compute_benchmark_range_from_pings(pings, range);
    compute_params_from_benchmark_range(range);
    track_img = cv::Mat(benchmark_nbr_rows, benchmark_nbr_cols, CV_8UC3, cv::Scalar(255, 255, 255));
}


void track_error_benchmark::track_img_params(const PointsT& points_maps, bool compute_range_from_points)
{
    benchmark_range range;
    // Compute range from the given PointsT& points_maps directly if the flag is set to true
    // Otherwise compute range from the gt_track set in the add_groundtruth method...
    if (compute_range_from_points) {
        compute_benchmark_range_from_pointsT(points_maps, range);
    } else {
        compute_benchmark_range_from_gt_track(range);
    }
    compute_params_from_benchmark_range(range);
    track_img = cv::Mat(benchmark_nbr_rows, benchmark_nbr_cols, CV_8UC3, cv::Scalar(255, 255, 255));
}


void track_error_benchmark::draw_track_img(mbes_ping::PingsT& pings, cv::Mat& img, const cv::Scalar& color, const std::string& name)
{
    //nbr_tracks_drawn += 1; // we should based the color on this instead

    double res, minx, miny, x0, y0;
    res = params[0]; minx = params[1]; miny = params[2]; x0 = params[3]; y0 = params[4];

    vector<cv::Point2f> curve_points;
    int counter = 0;
    for (const mbes_ping& ping : pings) {
        cv::Point2f pt(x0+res*(ping.pos_[0]-minx), img.rows-y0-res*(ping.pos_[1]-miny)-1);
        curve_points.push_back(pt);
        //cout << pt << endl;
        if (false) { //counter % 500 == 0) {
            double len = 30.;
            cv::Point pt1(x0+res*(ping.pos_[0]-minx), img.rows-y0-res*(ping.pos_[1]-miny)-1);
            cv::Point pt2(pt1.x + int(len*cos(ping.heading_)), pt1.y - int(len*sin(ping.heading_)));
            //cv::arrowedLine(img, pt1, pt2, cv::Scalar(0, 0 , 255), 2, 8, 0, 0.1);
            cv::putText(img, std::to_string(int(180./M_PI*ping.heading_)), pt1, cv::FONT_HERSHEY_PLAIN, 0.5, cv::Scalar(0, 0, 0), 1, 8, false);
        }
        ++counter;
    }

    cv::Mat curve(curve_points, true);
    curve.convertTo(curve, CV_32S); //adapt type for polylines
    cv::polylines(img, curve, false, color, 1, cv::LINE_AA);
    
    //track_img_path = "track.png";
}


void track_error_benchmark::draw_track_legend()
{

}

double track_error_benchmark::compute_rms_error(mbes_ping::PingsT& pings)
{
    if (pings.size() != gt_track.size()) {
        cout << "Track length not same as ground truth!" << endl;
        exit(-1);
    }

    double rms_error = 0.; double count = 0.;
    for (int i = 0; i < gt_track.size(); ++i) {
        rms_error += (pings[i].pos_-gt_track[i]).squaredNorm();
        count += 1.;
    }

    return sqrt(rms_error/count);
}

std::tuple<uint8_t, uint8_t, uint8_t> jet(double x)
{
    const double rone = 0.8;
    const double gone = 1.0;
    const double bone = 1.0;
    double r, g, b;

    x = (x < 0 ? 0 : (x > 1 ? 1 : x));

    if (x < 1. / 8.) {
        r = 0;
        g = 0;
        b = bone * (0.5 + (x) / (1. / 8.) * 0.5);
    } else if (x < 3. / 8.) {
        r = 0;
        g = gone * (x - 1. / 8.) / (3. / 8. - 1. / 8.);
        b = bone;
    } else if (x < 5. / 8.) {
        r = rone * (x - 3. / 8.) / (5. / 8. - 3. / 8.);
        g = gone;
        b = (bone - (x - 3. / 8.) / (5. / 8. - 3. / 8.));
    } else if (x < 7. / 8.) {
        r = rone;
        g = (gone - (x - 5. / 8.) / (7. / 8. - 5. / 8.));
        b = 0;
    } else {
        r = (rone - (x - 7. / 8.) / (1. - 7. / 8.) * 0.5);
        g = 0;
        b = 0;
    }

    return std::make_tuple(uint8_t(255.*r), uint8_t(255.*g), uint8_t(255.*b));
}

pair<double, cv::Mat> track_error_benchmark::compute_draw_consistency_map(mbes_ping::PingsT& pings)
{

    Eigen::MatrixXd means(benchmark_nbr_rows, benchmark_nbr_cols); means.setZero();
    Eigen::MatrixXd counts(benchmark_nbr_rows, benchmark_nbr_cols); counts.setZero();

    double res, minx, miny, x0, y0;
    res = params[0]; minx = params[1]; miny = params[2]; x0 = params[3]; y0 = params[4];

    for (const mbes_ping& ping : pings) {
        for (const Eigen::Vector3d& pos : ping.beams) {
            int col = int(x0+res*(pos[0]-minx));
            int row = int(y0+res*(pos[1]-miny));
            if (col >= 0 && col < benchmark_nbr_cols && row >= 0 && row < benchmark_nbr_rows) {
                means(row, col) += pos[2];
                counts(row, col) += 1.;
            }
        }
    }

    /*
    for (int i = 0; i < benchmark_nbr_rows; ++i) {
        for (int j = 0; j < benchmark_nbr_cols; ++j) {
            if (counts(i, j) == 0) {
                counts(i, j) = 1;
            }
        }
    }
    */

    Eigen::ArrayXXd bad = (counts.array() == 0.).cast<double>();
    counts.array() += bad;

    means.array() /= counts.array();
    
    Eigen::MatrixXd mean_offsets(benchmark_nbr_rows, benchmark_nbr_cols); mean_offsets.setZero();
    for (const mbes_ping& ping : pings) {
        for (const Eigen::Vector3d& pos : ping.beams) {
            int col = int(x0+res*(pos[0]-minx));
            int row = int(y0+res*(pos[1]-miny));
            if (col >= 0 && col < benchmark_nbr_cols && row >= 0 && row < benchmark_nbr_rows) {
                mean_offsets(row, col) += (pos[2] - means(row, col))*(pos[2] - means(row, col));
            }
        }
    }

    mean_offsets.array() /= counts.array();
    mean_offsets.array() = mean_offsets.array().sqrt();

    if (max_consistency_error == -1) {
        min_consistency_error = mean_offsets.minCoeff();
        max_consistency_error = mean_offsets.maxCoeff();
    }
    double maxv = max_consistency_error;
    double minv = min_consistency_error;
    double meanv = mean_offsets.mean();

    mean_offsets.array() -= minv;
    mean_offsets.array() /= (maxv - minv);

    cv::Mat error_img = cv::Mat(benchmark_nbr_rows, benchmark_nbr_cols, CV_8UC3, cv::Scalar(255, 255, 255));
    for (int i = 0; i < benchmark_nbr_rows; ++i) {
        for (int j = 0; j < benchmark_nbr_cols; ++j) {
            if (bad(i, j) == 1.) {
                continue;
            }
            cv::Point3_<uchar>* p = error_img.ptr<cv::Point3_<uchar> >(benchmark_nbr_rows-i-1, j);
            tie(p->z, p->y, p->x) = jet(mean_offsets(i, j));
        }
    }

    //cv::imshow("My image", error_img);
    //cv::waitKey();

    return make_pair(meanv, error_img);
}

void track_error_benchmark::map_draw_params(PointsT& map_points, PointsT& track_points,
                                            const int& submap_number){

    gt_track.clear();
    for(const Eigen::MatrixXd& track_i: track_points){
      for(unsigned int i=0; i<track_i.rows(); i++){
          gt_track.push_back(track_i.row(i));
      }
    }
    track_img_params(map_points);

    Eigen::MatrixXd means(benchmark_nbr_rows, benchmark_nbr_cols); means.setZero();
    Eigen::MatrixXd counts(benchmark_nbr_rows, benchmark_nbr_cols); counts.setZero();

    double res, minx, miny, x0, y0;
    res = params[0]; minx = params[1]; miny = params[2]; x0 = params[3]; y0 = params[4];

    for(const Eigen::MatrixXd& submap: map_points){
        for(unsigned int i = 0; i<submap.rows(); i++){
            Eigen::Vector3d pos = submap.row(i);
            int col = int(x0+res*(pos[0]-minx));
            int row = int(y0+res*(pos[1]-miny));
            if (col >= 0 && col < benchmark_nbr_cols && row >= 0 && row < benchmark_nbr_rows) {
                means(row, col) += pos[2];
                counts(row, col) += 1.;
            }
        }
    }

    Eigen::ArrayXXd counts_pos = counts.array() + (counts.array() == 0.).cast<double>();
    means.array() /= counts_pos;

    min_depth_ = means.minCoeff();
    means.array() -= min_depth_*(counts.array() > 0).cast<double>();
    max_depth_ = means.maxCoeff();
}

double track_error_benchmark::compute_vector_std(const std::vector<double>& vec, const double vec_mean) {
    double vec_std = 0;
    for (auto i : vec) {
        vec_std += pow(i - vec_mean, 2);
    }
    double denominator = vec.size() == 1? 1 : vec.size() - 1;
    return pow(1/(denominator)*vec_std, .5);
}

std::pair<double, Eigen::MatrixXd> track_error_benchmark::compute_grid_std(const std::vector<std::vector<std::vector<Eigen::MatrixXd>>>& grid_maps,
    int min_nbr_submap_hits, bool compute_mean) {

    double nbr_grids = 0.;
    double mean = 0;

    Eigen::MatrixXd standard_deviation(benchmark_nbr_rows, benchmark_nbr_cols); standard_deviation.setZero();
    for (int i = 0; i < benchmark_nbr_rows; ++i) {
        for (int j = 0; j < benchmark_nbr_cols; ++j) {
            int submap_with_hits = 0;
            std::vector<double> submap_means_at_grid;

            for (int k = 0; k < grid_maps[i][j].size(); ++k) {
                if (grid_maps[i][j][k].rows() > 0) {
                    submap_with_hits += 1;

                    if (compute_mean) {
                        std::vector<double> submap_hits_at_grid;
                        for (int p = 0; p <  grid_maps[i][j][k].rows(); p++) {
                            submap_hits_at_grid.push_back(grid_maps[i][j][k].row(p)[2]);
                        }
                        submap_means_at_grid.push_back(std::accumulate(submap_hits_at_grid.begin(),
                                                                    submap_hits_at_grid.end(),
                                                                    0.)/submap_hits_at_grid.size()
                        );
                    } else {
                        for (int p = 0; p <  grid_maps[i][j][k].rows(); p++) {
                            submap_means_at_grid.push_back(grid_maps[i][j][k].row(p)[2]);
                        }
                    }
                }
            }
            if (submap_with_hits < min_nbr_submap_hits) {
                continue;
            }
            nbr_grids += 1;
            mean = std::accumulate(submap_means_at_grid.begin(), submap_means_at_grid.end(), 0.) / submap_means_at_grid.size();
            standard_deviation(i, j) = compute_vector_std(submap_means_at_grid, mean);
        }
    }
    double average_std = standard_deviation.sum()/nbr_grids;
    cout << "Average std (" << min_nbr_submap_hits << ") = " << average_std << ", nbr grids = " << nbr_grids << endl;
    return make_pair(average_std, standard_deviation);
}
// Computes standard deviation of the grids with at least min_nbr_submap_hits
// The default behavior is to compute standard deviation of all grids with >= 1 hits, i.e.
// for grid cells with at least some hits by 1 submap
/*
std::pair<double, Eigen::MatrixXd> track_error_benchmark::compute_grid_std(const std::vector<std::vector<std::vector<double>>>& height_grid,
                                                                 const std::vector<std::vector<std::set<int>>>& hit_by_submaps,
                                                                 int min_nbr_submap_hits) {
    double nbr_grids = 0.;
    double mean = 0;
    Eigen::MatrixXd standard_deviation(benchmark_nbr_rows, benchmark_nbr_cols); standard_deviation.setZero();
    for (int i = 0; i < benchmark_nbr_rows; ++i) {
        for (int j = 0; j < benchmark_nbr_cols; ++j) {
            if (hit_by_submaps[i][j].size() < min_nbr_submap_hits) {
                continue;
            }
            nbr_grids += 1;
            mean = std::accumulate(height_grid[i][j].begin(), height_grid[i][j].end(), 0.) / height_grid[i][j].size();
            standard_deviation(i, j) = compute_vector_std(height_grid[i][j], mean);
        }
    }
    double average_std = standard_deviation.sum()/nbr_grids;
    cout << "Average std (" << min_nbr_submap_hits << ") = " << average_std << ", nbr grids = " << nbr_grids << endl;
    return make_pair(average_std, standard_deviation);
}
*/

cv::Mat track_error_benchmark::draw_height_map(const PointsT& points_maps, const std::string& name)
{
    Eigen::MatrixXd means(benchmark_nbr_rows, benchmark_nbr_cols); means.setZero();
    Eigen::MatrixXd counts(benchmark_nbr_rows, benchmark_nbr_cols); counts.setZero();

    double res, minx, miny, x0, y0;
    res = params[0]; minx = params[1]; miny = params[2]; x0 = params[3]; y0 = params[4];

    for(const Eigen::MatrixXd& submap: points_maps){
        for(unsigned int i = 0; i<submap.rows(); i++){
            Eigen::Vector3d pos = submap.row(i);
            int col = int(x0+res*(pos[0]-minx));
            int row = int(y0+res*(pos[1]-miny));
            if (col >= 0 && col < benchmark_nbr_cols && row >= 0 && row < benchmark_nbr_rows) {
                means(row, col) += pos[2];
                counts(row, col) += 1.;
            }
        }
    }

    Eigen::ArrayXXd counts_pos = counts.array() + (counts.array() == 0.).cast<double>();
    means.array() /= counts_pos;
    //TODO: refactor this!!!
    write_matrix_to_file(means, name);

    double minv = means.minCoeff();
    double meanv = means.mean();

    means.array() -= minv*(counts.array() > 0).cast<double>();
    double maxv = means.maxCoeff();
    means.array() /= maxv;

    cv::Mat mean_img = cv::Mat(benchmark_nbr_rows, benchmark_nbr_cols, CV_8UC3, cv::Scalar(255, 255, 255));
    for (int i = 0; i < benchmark_nbr_rows; ++i) {
        for (int j = 0; j < benchmark_nbr_cols; ++j) {
            if (means(i, j) == 0) {
                continue;
            }
            cv::Point3_<uchar>* p = mean_img.ptr<cv::Point3_<uchar> >(benchmark_nbr_rows-i-1, j);
            tie(p->z, p->y, p->x) = jet(means(i, j));
        }
    }

    return mean_img;
}

cv::Mat track_error_benchmark::draw_height_map(mbes_ping::PingsT& pings)
{
    Eigen::MatrixXd means(benchmark_nbr_rows, benchmark_nbr_cols); means.setZero();
    Eigen::MatrixXd counts(benchmark_nbr_rows, benchmark_nbr_cols); counts.setZero();

    double res, minx, miny, x0, y0;
    res = params[0]; minx = params[1]; miny = params[2]; x0 = params[3]; y0 = params[4];

    for (const mbes_ping& ping : pings) {
        for (const Eigen::Vector3d& pos : ping.beams) {
            int col = int(x0+res*(pos[0]-minx));
            int row = int(y0+res*(pos[1]-miny));
            if (col >= 0 && col < benchmark_nbr_cols && row >= 0 && row < benchmark_nbr_rows) {
                means(row, col) += pos[2];
                counts(row, col) += 1.;
            }
        }
    }

    Eigen::ArrayXXd counts_pos = counts.array() + (counts.array() == 0.).cast<double>();
    means.array() /= counts_pos;

    double minv = means.minCoeff();
    double meanv = means.mean();

    means.array() -= minv*(counts.array() > 0).cast<double>();
    double maxv = means.maxCoeff();
    means.array() /= maxv;

    cv::Mat mean_img = cv::Mat(benchmark_nbr_rows, benchmark_nbr_cols, CV_8UC3, cv::Scalar(255, 255, 255));
    for (int i = 0; i < benchmark_nbr_rows; ++i) {
        for (int j = 0; j < benchmark_nbr_cols; ++j) {
            if (means(i, j) == 0) {
                continue;
            }
            cv::Point3_<uchar>* p = mean_img.ptr<cv::Point3_<uchar> >(benchmark_nbr_rows-i-1, j);
            tie(p->z, p->y, p->x) = jet(means(i, j));
        }
    }

    return mean_img;
}

cv::Mat track_error_benchmark::draw_height_submap(PointsT& map_points, PointsT& track_points,
                                                  const int& submap_number){

    gt_track.clear();
    for(const Eigen::MatrixXd& track_i: track_points){
        for(unsigned int i=0; i<track_i.rows(); i++){
            gt_track.push_back(track_i.row(i));
        }
    }
    track_img_params(map_points);
    double res, minx, miny, x0, y0;
    res = params[0]; minx = params[1]; miny = params[2]; x0 = params[3]; y0 = params[4];

    Eigen::MatrixXd means(benchmark_nbr_rows, benchmark_nbr_cols); means.setZero();
    Eigen::MatrixXd counts(benchmark_nbr_rows, benchmark_nbr_cols); counts.setZero();
    for(const Eigen::MatrixXd& submap: map_points){
        for(unsigned int i = 0; i<submap.rows(); i++){
            Eigen::Vector3d pos = submap.row(i);
            int col = int(x0+res*(pos[0]-minx));
            int row = int(y0+res*(pos[1]-miny));
            if (col >= 0 && col < benchmark_nbr_cols && row >= 0 && row < benchmark_nbr_rows) {
                means(row, col) += pos[2];
                counts(row, col) += 1.;
            }
        }
    }

    Eigen::ArrayXXd counts_pos = counts.array() + (counts.array() == 0.).cast<double>();
    means.array() /= counts_pos;
    means.array() -= min_depth_*(counts.array() > 0).cast<double>();
    means.array() /= max_depth_;

    cv::Mat mean_img = cv::Mat(benchmark_nbr_rows, benchmark_nbr_cols, CV_8UC3, cv::Scalar(255, 255, 255));
    for (int i = 0; i < benchmark_nbr_rows; ++i) {
        for (int j = 0; j < benchmark_nbr_cols; ++j) {
            if (means(i, j) == 0) {
                continue;
            }
            cv::Point3_<uchar>* p = mean_img.ptr<cv::Point3_<uchar> >(benchmark_nbr_rows-i-1, j);
            tie(p->z, p->y, p->x) = jet(means(i, j));
        }
    }

    string mean_img_path = "submap_" + std::to_string(submap_number) + "_mean_depth.png";
    cv::imwrite(mean_img_path, mean_img);

    return mean_img;
}

void track_error_benchmark::add_ground_truth(mbes_ping::PingsT& pings)
{
    for (mbes_ping& ping : pings) {
        gt_track.push_back(ping.pos_);
    }
    track_img_params(pings);
    add_benchmark(pings, "ground_truth");
    track_img_path = dataset_name + "_benchmark_track_img.png";
}

void track_error_benchmark::add_ground_truth(const PointsT& map_points, const PointsT& track_points){
    for(const Eigen::MatrixXd& track_i: track_points){
        for(unsigned int i=0; i<track_i.rows(); i++){
            gt_track.push_back(track_i.row(i));
        }
    }
    track_img_params(map_points);
    add_benchmark(map_points, track_points, "ground_truth");
    track_img_path = dataset_name + "_benchmark_track_img.png";
}

void track_error_benchmark::add_initial(mbes_ping::PingsT& pings)
{
    input_pings = pings;
}

void track_error_benchmark::add_benchmark(const PointsT& maps_points, const PointsT& tracks_points,
                                          const std::string& name){
    cv::Mat error_img;
    Eigen::MatrixXd error_vals;
    double consistency_rms_error;
    std::vector<std::vector<std::vector<Eigen::MatrixXd> > > grid_maps = create_grids_from_matrices(maps_points);
    tie(consistency_rms_error, error_vals) = compute_consistency_error(grid_maps);
    error_img = draw_error_consistency_map(error_vals);
    std::string error_img_path = dataset_name + "_" + name + "_rms_consistency_error.png";
    cv::imwrite(error_img_path, error_img);
    error_img_paths[name] = error_img_path;
    consistency_rms_errors[name] = consistency_rms_error;
    write_matrix_to_file(error_vals, error_img_path);

    std::string mean_img_path = dataset_name + "_" + name + "_mean_depth.png";
    cv::Mat mean_img = draw_height_map(maps_points, mean_img_path);
    cv::imwrite(mean_img_path, mean_img);

    //Compute std error
    double average_std;
    Eigen::MatrixXd std_grids;
    std::vector<int> min_nbr_submap_hits{1, 2};
    std::vector<bool> use_mean{true, false};
    for (auto hits : min_nbr_submap_hits) {
        for (auto mean : use_mean) {
            tie(average_std, std_grids) = compute_grid_std(grid_maps, hits, mean);
            std_metrics[name][hits][mean] = average_std;
            std::string std_grids_img_path = dataset_name+"_"+name+"_std_" + std::to_string(hits)
                                             + "_use_mean_" + std::to_string(mean) + ".png";
            cv::imwrite(std_grids_img_path, draw_grid(std_grids));
            write_matrix_to_file(std_grids, std_grids_img_path);
        }
    }

    cout << " -------------- " << endl;
    cout << "Added benchmark " << name << endl;
    cout << "RMS consistency error: " << consistency_rms_error << endl;
    cout << "Consistency image map: " << error_img_path << endl;
    for (auto hits : min_nbr_submap_hits) {
        for (auto mean : use_mean) {
            cout << "Std (" << hits << ", use mean = " << mean << "): " << std_metrics[name][hits][mean] << endl;
        }
    }
    cout << " -------------- " << endl;

}

void track_error_benchmark::add_benchmark(mbes_ping::PingsT& pings, const std::string& name)
{
    int nbr_tracks_drawn = track_rms_errors.size();
    cv::Scalar color(colormap[nbr_tracks_drawn][2], colormap[nbr_tracks_drawn][1], colormap[nbr_tracks_drawn][0]);
    draw_track_img(pings, track_img, color, name);
    cv::Mat error_img;
    Eigen::MatrixXd error_vals;
    double consistency_rms_error;

    std::vector<std::vector<std::vector<Eigen::MatrixXd> > > grid_maps = create_grids_from_pings(pings);
    tie(consistency_rms_error, error_vals) = compute_consistency_error(grid_maps);
    error_img = draw_error_consistency_map(error_vals);
    draw_track_img(pings, error_img, cv::Scalar(0, 0, 0), name);
    string error_img_path = dataset_name + "_" + name + "_rms_consistency_error.png";
    cv::imwrite(error_img_path, error_img);
    error_img_paths[name] = error_img_path;
    consistency_rms_errors[name] = consistency_rms_error;
    
    cv::Mat mean_img = draw_height_map(pings);
    draw_track_img(pings, mean_img, cv::Scalar(0, 0, 0), name);
    string mean_img_path = dataset_name + "_" + name + "_mean_depth.png";
    cv::imwrite(mean_img_path, mean_img);

    double track_rms_error = compute_rms_error(pings);
    track_rms_errors[name] = track_rms_error;

    cout << "Added benchmark " << name << endl;
    cout << "RMS track error: " << track_rms_error << endl;
    cout << "RMS consistency error: " << consistency_rms_error << endl;
    cout << "Consistency image map: " << error_img_path << endl;

}

void track_error_benchmark::add_benchmark(pt_submaps::TransT& trans_corr, pt_submaps::RotsT& rots_corr, const std::string& name)
{
    mbes_ping::PingsT pings = input_pings;

    int i = 0;
    Eigen::Vector3d tc;
    Eigen::Matrix3d Rc;
    for (mbes_ping& ping : pings) {
        if (ping.first_in_file_) {
            tc = trans_corr[i];
            Rc = rots_corr[i];
            i += 1;
        }
        ping.pos_ = Rc*ping.pos_ + tc;
        for (Eigen::Vector3d& p : ping.beams) {
            p = Rc*p + tc;
        }
    }

    add_benchmark(pings, name);
}

void track_error_benchmark::print_summary()
{
    for (const pair<string, double>& p : track_rms_errors) {
        cout << p.first << " RMS track error: " << p.second << endl;
    }
    for (const pair<string, double>& p : consistency_rms_errors) {
        cout << p.first << " RMS consistency error: " << p.second << endl;
    }
    cout << "Max consistency error: " << max_consistency_error << endl;
    cout << "Min consistency error: " << min_consistency_error << endl;
}

void registration_summary_benchmark::print_summary()
{
    double optimized_error_sum = 0.;
    double initial_error_sum = 0.;
    double count = 0.;
    for (track_error_benchmark& benchmark : benchmarks) {
        for (const pair<string, double>& p : benchmark.consistency_rms_errors) {
            if (p.first.find("initial") != std::string::npos) {
                cout << "Adding benchmark " << p.first << " with error " << p.second << " to initial error..." << endl;
                initial_error_sum += p.second;
            }
            else if (p.first.find("optimized") != std::string::npos) {
                cout << "Adding benchmark " << p.first << " with error " << p.second << " to optimized error..." << endl;
                optimized_error_sum += p.second;
            }
        }
        count += 1.;
    }

    cout << "Initial mean RMS consistency error: " << initial_error_sum / count << endl;
    cout << "Optimized mean RMS consistency error: " << optimized_error_sum / count << endl;
}

void registration_summary_benchmark::add_registration_benchmark(mbes_ping::PingsT& initial_pings, mbes_ping::PingsT& optimized_pings, int i, int j)
{
    track_error_benchmark benchmark(dataset_name + "_registration_" + to_string(i) + " " + to_string(j));
    
    benchmark.add_ground_truth(initial_pings);

    benchmark.add_benchmark(initial_pings, "initial");
    benchmark.add_initial(initial_pings);

    benchmark.submap_origin = Eigen::Vector3d::Zero(); // this should be a method
    
    benchmark.add_benchmark(optimized_pings, "optimized");

    registration_pairs.push_back(make_pair(i, j));
    benchmarks.push_back(benchmark);
}

void registration_summary_benchmark::add_registration_benchmark(mbes_ping::PingsT& initial_pings, pt_submaps::TransT& trans_corr, pt_submaps::RotsT& rots_corr, int i, int j)
{
    mbes_ping::PingsT pings = initial_pings;

    int k = 0;
    Eigen::Vector3d tc;
    Eigen::Matrix3d Rc;
    for (mbes_ping& ping : pings) {
        if (ping.first_in_file_) {
            tc = trans_corr[k];
            Rc = rots_corr[k];
            k += 1;
        }
        ping.pos_ = Rc*ping.pos_ + tc;
        for (Eigen::Vector3d& p : ping.beams) {
            p = Rc*p + tc;
        }
    }

    add_registration_benchmark(initial_pings, pings, i, j);
}

mbes_ping::PingsT registration_summary_benchmark::get_submap_pings_pair(const mbes_ping::PingsT& pings, int i, int j)
{
    mbes_ping::PingsT pings_pair, pings_j;

    int k = 0;
    for (auto pos = pings.begin(); pos != pings.end(); ) {
        auto next = std::find_if(pos, pings.end(), [&](const mbes_ping& ping) {
            return ping.first_in_file_ && (&ping != &(*pos));
        });
        if (k == i) {
            pings_pair.insert(pings_pair.end(), pos, next);
        }
        else if (k == j) {
            pings_j.insert(pings_j.end(), pos, next);
        }
        ++k;
        pos = next;
    }
    pings_pair.insert(pings_pair.end(), pings_j.begin(), pings_j.end());

    return pings_pair;
}

mbes_ping::PingsT registration_summary_benchmark::get_submap_pings_index(const mbes_ping::PingsT& pings, int i)
{
    mbes_ping::PingsT pings_i;

    int k = 0;
    for (auto pos = pings.begin(); pos != pings.end(); ) {
        auto next = std::find_if(pos, pings.end(), [&](const mbes_ping& ping) {
            return ping.first_in_file_ && (&ping != &(*pos));
        });
        if (k == i) {
            pings_i.insert(pings_i.end(), pos, next);
            break;
        }
        ++k;
        pos = next;
    }

    return pings_i;
}

vector<vector<vector<Eigen::MatrixXd> > > track_error_benchmark::create_grids_from_pings(const mbes_ping::PingsT& pings){
    double res, minx, miny, x0, y0;
    res = params[0]; minx = params[1]; miny = params[2]; x0 = params[3]; y0 = params[4];

    int nbr_maps = std::accumulate(pings.begin(), pings.end(), 0, [](int sum, const mbes_ping& ping) {
        return sum + int(ping.first_in_file_);
    });

    vector<vector<vector<Eigen::MatrixXd> > > grid_maps(benchmark_nbr_rows);
    for (int i = 0; i < benchmark_nbr_rows; ++i) {
        grid_maps[i].resize(benchmark_nbr_cols);
        for (int j = 0; j < benchmark_nbr_cols; ++j) {
            grid_maps[i][j].resize(nbr_maps);
        }
    }

    int k = 0;
    // For each submap
    for (auto pos = pings.begin(); pos != pings.end(); ) {
        auto next = std::find_if(pos, pings.end(), [&](const mbes_ping& ping) {
            return ping.first_in_file_ && (&ping != &(*pos));
        });
        // For each ping in the submap
        for (auto iter = pos; iter < next; ++iter) {
            // For each beam in the ping
            for (const Eigen::Vector3d& point : iter->beams) {
                int col = int(x0+res*(point[0]-minx));
                int row = int(y0+res*(point[1]-miny));
                if (col >= 0 && col < benchmark_nbr_cols && row >= 0 && row < benchmark_nbr_rows) {
                    grid_maps[row][col][k].conservativeResize(grid_maps[row][col][k].rows()+1, 3);
                    grid_maps[row][col][k].bottomRows<1>() = point.transpose();
                }
            }
        }
        ++k;
        pos = next;
    }

    return grid_maps;
}

vector<vector<vector<Eigen::MatrixXd> > > track_error_benchmark::create_grids_from_matrices(const PointsT& points_maps){

    double res, minx, miny, x0, y0;
    res = params[0]; minx = params[1]; miny = params[2]; x0 = params[3]; y0 = params[4];

    int nbr_maps = points_maps.size();

    vector<vector<vector<Eigen::MatrixXd> > > grid_maps(benchmark_nbr_rows);
    for (int i = 0; i < benchmark_nbr_rows; ++i) {
        grid_maps[i].resize(benchmark_nbr_cols);
        for (int j = 0; j < benchmark_nbr_cols; ++j) {
            grid_maps[i][j].resize(nbr_maps);
        }
    }

    int k = 0;
    // For each submap
    for (const Eigen::MatrixXd& submap_k: points_maps) {
        // For each beam in submap k
        for(unsigned int i=0; i<submap_k.rows(); i++){
            const Eigen::Vector3d point_i = submap_k.row(i);
            int col = int(x0+res*(point_i[0]-minx));
            int row = int(y0+res*(point_i[1]-miny));
            if (col >= 0 && col < benchmark_nbr_cols && row >= 0 && row < benchmark_nbr_rows) {
                int nbr_points = grid_maps[row][col][k].rows() + 1;
                grid_maps[row][col][k].conservativeResize(nbr_points, 3);
                grid_maps[row][col][k].bottomRows<1>() = point_i.transpose();
            }
        }
        k++;
    }
    return grid_maps;
}

std::pair<double, Eigen::MatrixXd> track_error_benchmark::compute_consistency_error(
        const vector<vector<vector<Eigen::MatrixXd> > >& grid_maps)
{
    int nbr_maps = grid_maps[0][0].size();
    assert(("grid_maps.size() == benchmark_nbr_rows", grid_maps.size() == benchmark_nbr_rows));
    assert(("grid_maps[0].size() == benchmark_nbr_cols", grid_maps[0].size() == benchmark_nbr_cols));
//    cout << "Number maps for error benchmark: " << nbr_maps << endl;

    // Subsample grids
    Eigen::MatrixXd values(benchmark_nbr_rows, benchmark_nbr_cols); values.setZero();
/*
    //Eigen::MatrixXd counts(benchmark_nbr_rows, benchmark_nbr_cols); counts.setZero();
    for (int i = 0; i < benchmark_nbr_rows; ++i) {
        for (int j = 0; j < benchmark_nbr_cols; ++j) {
            for (int m = 0; m < nbr_maps; ++m) {
                if (grid_maps[i][j][m].rows() > 20) {
                    //cout << "map " << m << " size: " << grid_maps[i][j][m].rows() << endl;
                    int subsample = int(double(grid_maps[i][j][m].rows())/20.);
                    int counter = 0;
                    for (int p = 0; p < grid_maps[i][j][m].rows(); ++p) {
                        if (p % subsample == 0) {
                            grid_maps[i][j][m].row(counter) = grid_maps[i][j][m].row(p);
                            ++counter;
                        }
                    }
                    grid_maps[i][j][m].conservativeResize(counter, 3);
                }
            }
        }
    }
*/

    // For each grid
    double value_sum = 0.;
    double value_count = 0.;
    for (int i = 0; i < benchmark_nbr_rows; ++i) {
        for (int j = 0; j < benchmark_nbr_cols; ++j) {

            // Gather neighbors
            vector<Eigen::MatrixXd> neighborhood_points(nbr_maps);
            vector<bool> neighborhood_present(nbr_maps, false);
            for (int m = 0; m < nbr_maps; ++m) {
                for (int ii = std::max(i-1, 0); ii <= std::min(i+1, benchmark_nbr_rows-1); ++ii) {
                    for (int jj = std::max(j-1, 0); jj <= std::min(j+1, benchmark_nbr_cols-1); ++jj) {
                        if (grid_maps[ii][jj][m].rows() == 0) {
                            neighborhood_present[m] = false || neighborhood_present[m];
                            continue;
                        }
                        neighborhood_present[m] = true;
                        neighborhood_points[m].conservativeResize(neighborhood_points[m].rows() + grid_maps[ii][jj][m].rows(), 3);
                        neighborhood_points[m].bottomRows(grid_maps[ii][jj][m].rows()) = grid_maps[ii][jj][m];
                    }
                }
            }

            // Registration error
            double value = 0.;
            int nbr_averages = 10;
            for (int c = 0; c < nbr_averages; ++c) {
                double maxm = 0.;
                for (int m = 0; m < nbr_maps; ++m) {
                    if (grid_maps[i][j][m].rows() == 0) {
                        continue;
                    }
                    Eigen::Vector3d point = grid_maps[i][j][m].row(rand()%grid_maps[i][j][m].rows()).transpose();
//                    Eigen::Vector3d point = grid_maps[i][j][m].row(((int)grid_maps[i][j][m].rows()/10)*c).transpose();
                    for (int n = 0; n < nbr_maps; ++n) {
                        if (n == m || !neighborhood_present[n]) {
                            continue;
                        }
                        maxm = std::max((neighborhood_points[n].rowwise() - point.transpose()).rowwise().norm().minCoeff(), maxm);
                    }
                }
                value += maxm;
            }
            value /= double(nbr_averages);

            values(i, j) = value;
            if (value > 0) {
                value_sum += value*value;
                value_count += 1.;
            }
        }
    }

    return make_pair(sqrt(value_sum/value_count), values);
}


cv::Mat track_error_benchmark::draw_error_consistency_map(Eigen::MatrixXd values){

    Eigen::ArrayXXd bad = (values.array() == 0.).cast<double>();

    if (max_consistency_error == -1.) {
        max_consistency_error = values.maxCoeff();
        values.array() += max_consistency_error*bad;
        min_consistency_error = values.minCoeff();
    }
    double maxv = max_consistency_error;
    double minv = min_consistency_error;

    values.array() -= minv;
    values.array() /= (maxv - minv);

    cv::Mat error_img = cv::Mat(benchmark_nbr_rows, benchmark_nbr_cols, CV_8UC3, cv::Scalar(255, 255, 255));
    for (int i = 0; i < benchmark_nbr_rows; ++i) {
        for (int j = 0; j < benchmark_nbr_cols; ++j) {
            if (bad(i, j) == 1.) {
                continue;
            }
            cv::Point3_<uchar>* p = error_img.ptr<cv::Point3_<uchar> >(benchmark_nbr_rows-i-1, j);
            tie(p->z, p->y, p->x) = jet(values(i, j));
        }
    }

    return error_img;
}

cv::Mat track_error_benchmark::draw_grid(Eigen::MatrixXd values) {
    double maxv = values.maxCoeff();
    double minv = values.minCoeff();
    values.array() -= minv;
    values.array() /= (maxv - minv);
    cv::Mat img = cv::Mat(benchmark_nbr_rows, benchmark_nbr_cols, CV_8UC3, cv::Scalar(255, 255, 255));
    for (int i = 0; i < benchmark_nbr_rows; ++i) {
        for (int j = 0; j < benchmark_nbr_cols; ++j) {
            cv::Point3_<uchar>* p = img.ptr<cv::Point3_<uchar> >(benchmark_nbr_rows-i-1, j);
            if (values(i, j) == 0.) {
                continue;
            }
            tie(p->z, p->y, p->x) = jet(values(i, j));
        }
    }
    return img;
}
}
// namespace benchmark

namespace std_data {

using namespace benchmark;

template track_error_benchmark read_data<track_error_benchmark>(const boost::filesystem::path& path);
template void write_data<track_error_benchmark>(track_error_benchmark& data, const boost::filesystem::path& path);
template void write_data<registration_summary_benchmark>(registration_summary_benchmark& data, const boost::filesystem::path& path);

}
