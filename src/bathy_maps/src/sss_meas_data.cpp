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

#include <bathy_maps/sss_meas_data.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

sss_meas_data_builder::sss_meas_data_builder(const sss_meas_data::BoundsT& bounds, double resolution, int nbr_pings) : 
    waterfall_width(2*nbr_pings), waterfall_counter(0)
{
    global_origin = Eigen::Vector3d(bounds(0, 0), bounds(0, 1), 0.);

    /*
    image_cols = resolution*(bounds(1, 0) - bounds(0, 0));
    image_rows = resolution*(bounds(1, 1) - bounds(0, 1));

    sss_meas_data_counts = Eigen::MatrixXd::Zero(image_rows, image_cols);
    sss_meas_data_sums = Eigen::MatrixXd::Zero(image_rows, image_cols);
    */

    sss_waterfall_image = Eigen::MatrixXd::Zero(2000, waterfall_width);
    sss_waterfall_hits_X = Eigen::MatrixXd::Zero(2000, waterfall_width);
    sss_waterfall_hits_Y = Eigen::MatrixXd::Zero(2000, waterfall_width);
    sss_waterfall_hits_Z = Eigen::MatrixXd::Zero(2000, waterfall_width);
}

size_t sss_meas_data_builder::get_waterfall_bins()
{
    return waterfall_width / 2;
}

bool sss_meas_data_builder::empty()
{
    return waterfall_counter == 0;
}

Eigen::MatrixXd sss_meas_data_builder::downsample_cols(const Eigen::MatrixXd& M, int new_cols)
{
    if (new_cols == M.cols()) {
        return M;
    }

    double factor = double(new_cols)/double(M.cols());
    Eigen::ArrayXXd sums = Eigen::MatrixXd::Zero(M.rows(), new_cols);
    Eigen::ArrayXXd counts = Eigen::MatrixXd::Zero(M.rows(), new_cols);

    int ind;
    for (int i = 0; i < M.rows(); ++i) {
        for (int j = 0; j < M.cols(); ++j) {
            ind = int(factor*j);
            if (M(i, j) != 0) {
                sums(i, ind) += M(i, j);
                counts(i, ind) += 1.;
            }
        }
    }

    counts += (counts == 0).cast<double>();
    sums /= counts;

    return sums.matrix();
}

sss_meas_data sss_meas_data_builder::finish()
{
    sss_meas_data meas_data;
    meas_data.pos = pos;
    meas_data.rpy = rpy;
    meas_data.ping_id = ping_id;
    if (waterfall_width == 512) {
        meas_data.sss_waterfall_image = sss_waterfall_image.topRows(waterfall_counter).cast<float>();
        meas_data.sss_waterfall_hits_X = sss_waterfall_hits_X.topRows(waterfall_counter).cast<float>();
        meas_data.sss_waterfall_hits_Y = sss_waterfall_hits_Y.topRows(waterfall_counter).cast<float>();
        meas_data.sss_waterfall_hits_Z = sss_waterfall_hits_Z.topRows(waterfall_counter).cast<float>();
    }
    else {
        meas_data.sss_waterfall_image = downsample_cols(sss_waterfall_image.topRows(waterfall_counter), 512).cast<float>();
        meas_data.sss_waterfall_hits_X = downsample_cols(sss_waterfall_hits_X.topRows(waterfall_counter), 512).cast<float>();
        meas_data.sss_waterfall_hits_Y = downsample_cols(sss_waterfall_hits_Y.topRows(waterfall_counter), 512).cast<float>();
        meas_data.sss_waterfall_hits_Z = downsample_cols(sss_waterfall_hits_Z.topRows(waterfall_counter), 512).cast<float>();
    }

    return meas_data;
}

// if I were to just send in the inds instead, that
// would be quite a bit easier, I wouldn't have to send
// in e.g. the depths
void sss_meas_data_builder::add_hits(const Eigen::MatrixXd& hits, const Eigen::VectorXi& hits_inds,
                                     const Eigen::VectorXd& intensities,
                                     const Eigen::VectorXd& sss_depths, const Eigen::VectorXd& sss_model,
                                     const std_data::sss_ping_side& ping, const Eigen::Vector3d& current_pos,
                                     const Eigen::Vector3d& current_rpy, bool is_left)
{
    if (!is_left) {
        ping_id.push_back(waterfall_counter);
        pos.push_back(current_pos);
        rpy.push_back(current_rpy);
    }

    // this might be the culprit
    if (hits.rows() == 0) {
        if (!is_left) {
            ++waterfall_counter;
        }
        return;
    }

    /*
    // this might be the culprit
    if (hits.rows() == 0) {
        return;
    }

    pos.push_back(current_pos);
    rpy.push_back(current_rpy);
    */

    std::cout << "Hits rows: " << hits.rows() << std::endl;
    std::cout << "Origin: " << global_origin.transpose() << ", pose: " << current_pos.transpose() << std::endl;

    if (waterfall_counter >= sss_waterfall_image.rows()) {
        sss_waterfall_image.conservativeResize(sss_waterfall_image.rows()+1000, sss_waterfall_image.cols());
        sss_waterfall_hits_X.conservativeResize(sss_waterfall_image.rows()+1000, sss_waterfall_image.cols());
        sss_waterfall_hits_Y.conservativeResize(sss_waterfall_image.rows()+1000, sss_waterfall_image.cols());
        sss_waterfall_hits_Z.conservativeResize(sss_waterfall_image.rows()+1000, sss_waterfall_image.cols());
    }

    if (waterfall_width == 512) {
        double ping_step = double(ping.pings.size()) / double(waterfall_width/2);
        Eigen::ArrayXd value_windows = Eigen::VectorXd::Zero(waterfall_width/2);
        Eigen::ArrayXd value_counts = Eigen::ArrayXd::Zero(waterfall_width/2);
        for (int i = 0; i < ping.pings.size(); ++i) {
            int col = int(double(i)/ping_step);
            value_windows(col) += double(ping.pings[i])/10000.;
            value_counts(col) += 1.;
        }
        value_counts += (value_counts == 0).cast<double>();
        if (is_left) {
            sss_waterfall_image.block(waterfall_counter, waterfall_width/2, 1, waterfall_width/2) = (value_windows / value_counts).transpose();
        }
        else {
            sss_waterfall_image.block(waterfall_counter, 0, 1, waterfall_width/2) = (value_windows / value_counts).reverse().transpose();
        }
    }
    else {
        for (int i = 0; i < ping.pings.size(); ++i) {
            int col;
            if (is_left) {
                col = waterfall_width/2 + i;
            }
            else {
                col = waterfall_width/2 - 1 - i;
            }
            sss_waterfall_image(waterfall_counter, col) = double(ping.pings[i])/10000.;
        }
    }

    for (int i = 0; i < hits.rows(); ++i) {
        int ind = hits_inds(i);
        int col;
        if (ind == -1) {
            continue;
        }
        else if (is_left) {
            col = waterfall_width/2 + ind;
        }
        else {
            col = waterfall_width/2 - 1 - ind;
        }
        sss_waterfall_hits_X(waterfall_counter, col) = hits(i, 0);
        sss_waterfall_hits_Y(waterfall_counter, col) = hits(i, 1);
        sss_waterfall_hits_Z(waterfall_counter, col) = hits(i, 2);
    }

    /*
    if (is_left) {
        sss_waterfall_depth.block(waterfall_counter, waterfall_width/2, 1, waterfall_width/2) = sss_depths.transpose();
        sss_waterfall_model.block(waterfall_counter, waterfall_width/2, 1, waterfall_width/2) = sss_model.transpose();
    }
    else {
        sss_waterfall_depth.block(waterfall_counter, 0, 1, waterfall_width/2) = sss_depths.reverse().transpose();
        sss_waterfall_model.block(waterfall_counter, 0, 1, waterfall_width/2) = sss_model.reverse().transpose();
    }
    */

    if (!is_left) {
        ++waterfall_counter;
    }
}
