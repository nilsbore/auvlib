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

#include <bathy_maps/base_draper.h>

#include <igl/per_face_normals.h>
#include <bathy_maps/mesh_map.h>
#include <sonar_tracing/snell_ray_tracing.h>
#include <chrono>

using namespace std;
using namespace xtf_data;
using namespace csv_data;

BaseDraper::BaseDraper(const Eigen::MatrixXd& V1, const Eigen::MatrixXi& F1,
                       const BoundsT& bounds,
                       const csv_asvp_sound_speed::EntriesT& sound_speeds)
    : V1(V1), F1(F1),
      sound_speeds(sound_speeds), bounds(bounds),
      sensor_yaw(0.), ray_tracing_enabled(false),
      tracing_map_size(200.), intensity_multiplier(1.)
{
    offset = Eigen::Vector3d(bounds(0, 0), bounds(0, 1), 0.);
    sensor_offset_port = Eigen::Vector3d::Zero();
    sensor_offset_stbd = Eigen::Vector3d::Zero();
    igl::per_face_normals(V1, F1, N1); // compute normals for mesh 
}

void BaseDraper::set_ray_tracing_enabled(bool enabled)
{
    ray_tracing_enabled = enabled;
}

pair<Eigen::Vector3d, Eigen::Vector3d> BaseDraper::get_port_stbd_sensor_origins(const std_data::sss_ping& ping)
{
    Eigen::Matrix3d Ry = Eigen::AngleAxisd(ping.pitch_, Eigen::Vector3d::UnitY()).matrix();
    Eigen::Matrix3d Rz = Eigen::AngleAxisd(ping.heading_, Eigen::Vector3d::UnitZ()).matrix();
    Eigen::Vector3d offset_pos = ping.pos_ - offset;
    Eigen::Vector3d origin_port = offset_pos + Rz*Ry*sensor_offset_port;
    Eigen::Vector3d origin_stbd = offset_pos + Rz*Ry*sensor_offset_stbd;
    return make_pair(origin_port, origin_stbd);
}

pair<Eigen::MatrixXd, Eigen::MatrixXd> BaseDraper::compute_sss_dirs(const Eigen::Matrix3d& R, double tilt_angle, double beam_width, int nbr_lines)
{
    const double min_theta = tilt_angle - 0.5*beam_width; // M_PI/180.*10.;
    const double max_theta = tilt_angle + 0.5*beam_width; //M_PI/180.*60.;

    double min_c = 1./cos(min_theta);
    double max_c = 1./cos(max_theta);
    double step = (max_c - min_c)/double(nbr_lines-1);

    Eigen::MatrixXd dirs_left(nbr_lines, 3);
    Eigen::MatrixXd dirs_right(nbr_lines, 3);
    for (int i = 0; i < nbr_lines; ++i) {
        double ci = min_c + double(i)*step;
        double bi = sqrt(ci*ci-1.);
        Eigen::Vector3d dir_left(0., bi, -1.);
        Eigen::Vector3d dir_right(0., -bi, -1.);
        dirs_left.row(i) = (R*dir_left).transpose();
        dirs_right.row(i) = (R*dir_right).transpose();
    }

    return make_pair(dirs_left, dirs_right);
}

tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd> BaseDraper::project(const std_data::sss_ping& ping)
{
    if (DEBUG_OUTPUT) cout << "Setting new position: " << ping.pos_.transpose() << endl;
    Eigen::Matrix3d Rcomp = Eigen::AngleAxisd(sensor_yaw, Eigen::Vector3d::UnitZ()).matrix();
    Eigen::Matrix3d Ry = Eigen::AngleAxisd(ping.pitch_, Eigen::Vector3d::UnitY()).matrix();
    Eigen::Matrix3d Rz = Eigen::AngleAxisd(ping.heading_, Eigen::Vector3d::UnitZ()).matrix();
    Eigen::Matrix3d R = Rz*Ry*Rcomp;
    Eigen::Vector3d offset_pos = ping.pos_ - offset;

    Eigen::MatrixXd hits_left;
    Eigen::MatrixXd normals_left;
    Eigen::MatrixXd hits_right;
    Eigen::MatrixXd normals_right;

    // all of this is for adaptively determining the beam_width and tilt_angle for a certain depth
    auto start = chrono::high_resolution_clock::now();
    double depth = .8*tracer.depth_mesh_underneath_vehicle(offset_pos, V1, F1); // make it slightly wider
    if (depth == 0.) {
        return make_tuple(hits_left, hits_right, normals_left, normals_right);
    }
    Eigen::VectorXd speeds, depths;
    tie(speeds, depths) = get_sound_vels_below(offset_pos);
    double max_distance = .5*speeds.mean()*ping.port.time_duration;
    double beam_width = acos(depth/max_distance);
    double tilt_angle = beam_width/2.;
    auto stop = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::microseconds>(stop - start);
    if (DEBUG_OUTPUT) cout << "depth_mesh_underneath_vehicle time: " << duration.count() << " microseconds" << endl;

    Eigen::Vector3d origin_port;
    Eigen::Vector3d origin_stbd;
    tie(origin_port, origin_stbd) = get_port_stbd_sensor_origins(ping);

    Eigen::MatrixXd dirs_left;
    Eigen::MatrixXd dirs_right;
    tie(dirs_left, dirs_right) = compute_sss_dirs(R, tilt_angle, beam_width, 500);

    tie(hits_left, normals_left) = trace_side(ping.port, origin_port, dirs_left);
    tie(hits_right, normals_right) = trace_side(ping.stbd, origin_stbd, dirs_right);

    return make_tuple(hits_left, hits_right, normals_left, normals_right);
}

// New style functions follow here:
tuple<Eigen::MatrixXd, Eigen::MatrixXd> BaseDraper::trace_side(const std_data::sss_ping_side& ping,
                                                               const Eigen::Vector3d& sensor_origin,
                                                               const Eigen::MatrixXd& dirs)
{
    Eigen::MatrixXd hits;
    Eigen::VectorXi hits_inds;
    Eigen::MatrixXd normals;

    auto start = chrono::high_resolution_clock::now();
    
    tie(hits, hits_inds) = tracer.compute_hits(sensor_origin, dirs, V1, F1);
    auto stop = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::microseconds>(stop - start);
    if (DEBUG_OUTPUT) cout << "embree_compute_hits full time: " << duration.count() << " microseconds" << endl;
    
    normals.resize(hits.rows(), 3);
    for (int j = 0; j < hits.rows(); ++j) {
        normals.row(j) = N1.row(hits_inds(j));
    }

    return make_tuple(hits, normals);
}

/*
double BaseDraper::compute_simple_sound_vel()
{
    double sound_vel = sound_speeds[0].vels.head(sound_speeds[0].vels.rows()-1).mean();
    return sound_vel;
}
*/

pair<Eigen::VectorXd, Eigen::VectorXd> BaseDraper::get_sound_vels_below(const Eigen::Vector3d& sensor_origin)
{
    int i;
    int nbr_speeds = sound_speeds[0].dbars.rows();
    for (i = 0; i < nbr_speeds; ++i) {
        if (-sound_speeds[0].dbars[i] < sensor_origin(2)) {
            if (DEBUG_OUTPUT) cout << "Breaking at " << -sound_speeds[0].dbars[i] << ", which is below " << sensor_origin(2) << endl;
            break;
        }
    }

    if (i >= nbr_speeds) {
        return make_pair(sound_speeds[0].vels.tail(1), (sensor_origin(2)-1.)*Eigen::VectorXd::Ones(1));
    }

    return make_pair(sound_speeds[0].vels.tail(nbr_speeds-i), -sound_speeds[0].dbars.tail(nbr_speeds-i));
}

Eigen::VectorXd BaseDraper::compute_times(const Eigen::Vector3d& sensor_origin, const Eigen::MatrixXd& P)
{
    if (ray_tracing_enabled) {
        return compute_refraction_times(sensor_origin, P);
    }

    Eigen::VectorXd speeds, depths;
    tie(speeds, depths) = get_sound_vels_below(sensor_origin);
    //double sound_vel = compute_simple_sound_vel();
    Eigen::VectorXd times = 2.*(P.rowwise() - sensor_origin.transpose()).rowwise().norm()/speeds.mean();
    return times;
}

Eigen::VectorXd BaseDraper::compute_refraction_times(const Eigen::Vector3d& sensor_origin, const Eigen::MatrixXd& P)
{
    static bool is_left = true;

    /*
    Eigen::VectorXd layer_depths = -sound_speeds[0].dbars.segment(1, sound_speeds[0].dbars.rows()-2);
    Eigen::VectorXd layer_speeds = sound_speeds[0].vels.head(sound_speeds[0].vels.rows()-1);
    */
    Eigen::VectorXd layer_speeds, layer_depths;
    tie(layer_speeds, layer_depths) = get_sound_vels_below(sensor_origin);
    layer_depths.array() -= sensor_origin(2); // refraction assumes sensor z=0

    if (DEBUG_OUTPUT) cout << "Number of depths: " << layer_depths.rows() << ", number of speeds: " << layer_speeds.rows() << endl;

    Eigen::VectorXd x = (P.leftCols<2>().rowwise() - sensor_origin.head<2>().transpose()).rowwise().norm();
    Eigen::MatrixXd end_points(x.rows(), 2);
    end_points.col(0) = x;
    end_points.col(1) = P.col(2).array() - sensor_origin(2); // refraction assumes sensor z=0

    Eigen::MatrixXd layer_widths;
    Eigen::VectorXd times;
    tie(times, layer_widths) = trace_multiple_layers(layer_depths, layer_speeds, end_points);
    times.array() *= 2.; // back and forth
    if (DEBUG_OUTPUT) cout << "Got final times: " << times.transpose() << endl;

    visualize_rays(end_points, layer_depths, layer_widths, -25., false, is_left);

    is_left = !is_left;

    return times;
}

Eigen::VectorXi BaseDraper::compute_bin_indices(const Eigen::VectorXd& times, const std_data::sss_ping_side& ping, size_t nbr_windows)
{
    Eigen::VectorXi bin_inds = Eigen::VectorXi::Zero(times.rows());

    double ping_step = ping.time_duration / double(nbr_windows);
    for (int i = 0; i < times.rows(); ++i) {
        int index = int(times(i)/ping_step);
        if (index < nbr_windows) {
            bin_inds(i) = index;
        }
        else {
            bin_inds(i) = -1;
        }
    }

    return bin_inds;
}

Eigen::VectorXd BaseDraper::convert_to_time_bins(const Eigen::VectorXd& times, const Eigen::VectorXd& values,
                                                 const std_data::sss_ping_side& ping, size_t nbr_windows)
{
    double ping_step = ping.time_duration / double(nbr_windows);
    Eigen::VectorXd value_windows = Eigen::VectorXd::Zero(nbr_windows);
    Eigen::ArrayXd value_counts = Eigen::ArrayXd::Zero(nbr_windows);
    for (int i = 0; i < times.rows(); ++i) {
        int index = int(times(i)/ping_step);
        if (index < nbr_windows) {
            value_windows(index) += values(i);
            value_counts(index) += 1.;
        }
    }
    value_counts += (value_counts == 0).cast<double>();
    value_windows.array() /= value_counts;

    return value_windows;
}

Eigen::MatrixXd BaseDraper::convert_to_time_bins(const Eigen::VectorXd& times, const Eigen::MatrixXd& values,
                                                 const std_data::sss_ping_side& ping, size_t nbr_windows)
{
    double ping_step = ping.time_duration / double(nbr_windows);
    Eigen::MatrixXd value_windows = Eigen::MatrixXd::Zero(nbr_windows, values.cols());
    Eigen::ArrayXXd value_counts = Eigen::ArrayXXd::Zero(nbr_windows, values.cols());
    for (int i = 0; i < times.rows(); ++i) {
        int index = int(times(i)/ping_step);
        if (index < nbr_windows) {
            value_windows.row(index) += values.row(i);
            value_counts.row(index) += 1.;
        }
    }
    value_counts += (value_counts == 0).cast<double>();
    value_windows.array() /= value_counts;

    return value_windows;
}

Eigen::VectorXd BaseDraper::compute_intensities(const Eigen::VectorXd& times, 
                                                const std_data::sss_ping_side& ping)
{
    double ping_step = ping.time_duration / double(ping.pings.size());

    Eigen::VectorXd intensities = Eigen::VectorXd::Zero(times.size());
    for (int i = 0; i < times.rows(); ++i) {
        int ping_index = int(times(i)/ping_step);
        if (ping_index < ping.pings.size()) {
            intensities(i) += double(ping.pings[ping_index])/10000.;
        }
    }
    return intensities;
}

Eigen::VectorXd BaseDraper::compute_lambert_intensities(const Eigen::MatrixXd& hits, const Eigen::MatrixXd& normals,
                                                        const Eigen::Vector3d& origin)
{
    Eigen::VectorXd intensities(hits.rows());

    for (int j = 0; j < hits.rows(); ++j) { 
        Eigen::Vector3d dir = origin - hits.row(j).transpose();
        double dist = dir.norm();
        dir.normalize();
        Eigen::Vector3d n = normals.row(j).transpose();
        n.normalize();
        //intensities(j) = std::min(fabs(dir.dot(n))*(300./(dist*dist)), 1.);
        intensities(j) = std::min(std::max(fabs(dir.dot(n)), 0.), 1.);
        intensities(j) = intensities(j)*intensities(j);
    }

    return intensities;
}

Eigen::VectorXd BaseDraper::compute_model_intensities(const Eigen::VectorXd& dists, const Eigen::VectorXd& thetas)
{
    Eigen::VectorXd intensities(dists.rows());

    double alpha = 0.5;
    double sigma_theta = 0.3;

    std::normal_distribution<double> noise_dist(1., sigma_theta);

    for (int j = 0; j < dists.rows(); ++j) { 
        double dist = dists(j);
        double theta = thetas(j);
        double TL = 20.*log10(dist); //1./(dist*dist);
        double DL = cos(theta);
        double G = std::min(1., 2.*DL*DL);
        double SL = G/DL*exp(-theta*theta/(2.*sigma_theta*sigma_theta));
        double SS = 10.*log10((1. - alpha)*DL + alpha*SL);
        double NL = 10.*log10(noise_dist(generator));
        //intensities(j) = 1./(-25.+42.)*(42. + SS - TL + NL); // log(1.+1.73*200.*TL*SS*NL);
        intensities(j) = 1./(10.)*(9. + SS + NL); // log(1.+1.73*200.*TL*SS*NL);
        intensities(j) = std::min(std::max(intensities(j), 0.), 1.);
        //intensities(j) = DL*DL;
        //10*log(1.73*200.)-log(TL)+log(SS)+NL
    }

    return intensities;
}

Eigen::VectorXd BaseDraper::compute_model_intensities(const Eigen::MatrixXd& hits, const Eigen::MatrixXd& normals,
                                                      const Eigen::Vector3d& origin)
{
    Eigen::VectorXd thetas(hits.rows());
    Eigen::VectorXd dists(hits.rows());

    for (int j = 0; j < hits.rows(); ++j) { 
        Eigen::Vector3d dir = origin - hits.row(j).transpose();
        double dist = dir.norm();
        dir.normalize();
        Eigen::Vector3d n = normals.row(j).transpose();
        n.normalize();
        double theta = acos(dir.dot(n));
        thetas(j) = theta;
        dists(j) = dist;
    }

    return compute_model_intensities(dists, thetas);
}

sss_draping_result BaseDraper::project_ping(const std_data::sss_ping& ping, int nbr_bins)
{
    Eigen::MatrixXd hits_left;
    Eigen::MatrixXd hits_right;
    Eigen::MatrixXd normals_left;
    Eigen::MatrixXd normals_right;
    auto start = chrono::high_resolution_clock::now();
    tie(hits_left, hits_right, normals_left, normals_right) = project(ping);
    auto stop = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::microseconds>(stop - start);
    if (DEBUG_OUTPUT) cout << "project time: " << duration.count() << " microseconds" << endl;

    start = chrono::high_resolution_clock::now();
    // these should take care of computing bending if set
    // TODO: add origin to compute times
    Eigen::Vector3d origin_port;
    Eigen::Vector3d origin_stbd;
    tie(origin_port, origin_stbd) = get_port_stbd_sensor_origins(ping);
    stop = chrono::high_resolution_clock::now();
    duration = chrono::duration_cast<chrono::microseconds>(stop - start);
    if (DEBUG_OUTPUT) cout << "get_port_stbd_sensor_origins time: " << duration.count() << " microseconds" << endl;

    start = chrono::high_resolution_clock::now();
    ping_draping_result left = project_ping_side(ping.port, hits_left, normals_left, origin_port, nbr_bins);
    stop = chrono::high_resolution_clock::now();
    duration = chrono::duration_cast<chrono::microseconds>(stop - start);
    if (DEBUG_OUTPUT) cout << "project_ping_side left time: " << duration.count() << " microseconds" << endl;
    start = chrono::high_resolution_clock::now();
    ping_draping_result right = project_ping_side(ping.stbd, hits_right, normals_right, origin_stbd, nbr_bins);
    stop = chrono::high_resolution_clock::now();
    duration = chrono::duration_cast<chrono::microseconds>(stop - start);
    if (DEBUG_OUTPUT) cout << "project_ping_side right time: " << duration.count() << " microseconds" << endl;

    return make_pair(left, right);
}

ping_draping_result BaseDraper::project_ping_side(const std_data::sss_ping_side& sensor, const Eigen::MatrixXd& hits,
                                                  const Eigen::MatrixXd& hits_normals, const Eigen::Vector3d& origin,
                                                  int nbr_bins)
{
    ping_draping_result res;
    res.hits_points = hits;
    res.sensor_origin = origin;

    res.hits_times = compute_times(origin, hits);

    // compute the elevation waterfall row
    res.time_bin_points = convert_to_time_bins(res.hits_times, hits, sensor, nbr_bins);
    res.time_bin_normals = convert_to_time_bins(res.hits_times, hits_normals, sensor, nbr_bins);

    // compute the intensities of the model
    Eigen::VectorXd model_intensities = compute_model_intensities(hits, hits_normals, origin);
    res.time_bin_model_intensities = convert_to_time_bins(res.hits_times, model_intensities, sensor, nbr_bins);

    // compute the ground truth intensities
    res.hits_intensities = compute_intensities(res.hits_times, sensor);

    // compute waterfall image inds of hits
    res.hits_inds = compute_bin_indices(res.hits_times, sensor, nbr_bins);

    if (DEBUG_OUTPUT) cout << "Adding hits: " << res.hits_points.rows() << endl;

    return res;
}

Eigen::VectorXd compute_bin_intensities(const std_data::sss_ping_side& ping, int nbr_bins)
{
    auto start = chrono::high_resolution_clock::now();
    double ping_step = double(ping.pings.size()) / double(nbr_bins);

    Eigen::VectorXd intensities = Eigen::VectorXd::Zero(nbr_bins);
    Eigen::ArrayXd counts = Eigen::ArrayXd::Zero(nbr_bins);
    for (int i = 0; i < ping.pings.size(); ++i) {
        int intensity_index = int(double(i)/ping_step);
        if (intensity_index < intensities.rows()) {
            intensities(intensity_index) += double(ping.pings[i])/10000.;
            counts(intensity_index) += 1.;
        }
    }
    counts += (counts == 0).cast<double>();
    intensities.array() /= counts;
    auto stop = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::microseconds>(stop - start);
    if (DEBUG_OUTPUT) cout << "compute_bin_intensities time: " << duration.count() << " microseconds" << endl;
    return intensities;
}
