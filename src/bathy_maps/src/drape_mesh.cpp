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

#include <bathy_maps/drape_mesh.h>

#include <igl/ray_mesh_intersect.h>
#include <igl/barycentric_to_global.h>
#include <igl/embree/line_mesh_intersection.h>

#include <sonar_tracing/snell_ray_tracing.h>

using namespace std;
using namespace xtf_data;
using namespace csv_data;

pair<Eigen::MatrixXd, Eigen::MatrixXd> compute_sss_dirs(const Eigen::Matrix3d& R, double tilt_angle, double beam_width, int nbr_lines)
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

tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXi, Eigen::MatrixXi> compute_hits(const Eigen::Vector3d& origin, const Eigen::Matrix3d& R, double tilt_angle, double beam_width, const Eigen::MatrixXd& V1, const Eigen::MatrixXi& F1)
{
    igl::Hit hit;
    Eigen::MatrixXd dirs_left;
    Eigen::MatrixXd dirs_right;
    tie(dirs_left, dirs_right) = compute_sss_dirs(R, tilt_angle, beam_width, 220);
    Eigen::MatrixXd hits_left(dirs_left.rows(), 3);
    Eigen::MatrixXd hits_right(dirs_right.rows(), 3);
    Eigen::VectorXi hits_left_inds(dirs_left.rows());
    Eigen::VectorXi hits_right_inds(dirs_right.rows());
    int hit_count = 0;
    for (int i = 0; i < dirs_left.rows(); ++i) {
        bool did_hit = ray_mesh_intersect(origin, dirs_left.row(i).transpose(), V1, F1, hit);
        if (did_hit) {
            int vind = F1(hit.id, 0);
            hits_left_inds(hit_count) = hit.id;
            // we actually get the coordinates within the triangle also, we could use that
            hits_left.row(hit_count) = V1.row(vind);
            ++hit_count;
        }
    }
    hits_left.conservativeResize(hit_count, 3);
    hits_left_inds.conservativeResize(hit_count);
    hit_count = 0;
    for (int i = 0; i < dirs_right.rows(); ++i) {
        bool did_hit = ray_mesh_intersect(origin, dirs_right.row(i).transpose(), V1, F1, hit);
        if (did_hit) {
            int vind = F1(hit.id, 0);
            hits_right_inds(hit_count) = hit.id;
            // we actually get the coordinates within the triangle also, we could use that
            hits_right.row(hit_count) = V1.row(vind);
            ++hit_count;
        }
    }
    hits_right.conservativeResize(hit_count, 3);
    hits_right_inds.conservativeResize(hit_count);
    return make_tuple(hits_left, hits_right, hits_left_inds, hits_right_inds);
}

tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXi, Eigen::MatrixXi, Eigen::VectorXd, Eigen::VectorXd> embree_compute_hits(const Eigen::Vector3d& origin, const Eigen::Matrix3d& R, double tilt_angle, double beam_width, const Eigen::MatrixXd& V1, const Eigen::MatrixXi& F1)
{
    auto start = chrono::high_resolution_clock::now();
    igl::Hit hit;
    Eigen::MatrixXd dirs_left;
    Eigen::MatrixXd dirs_right;
    //tie(dirs_left, dirs_right) = compute_sss_dirs(R, tilt_angle, beam_width, 220);
    tie(dirs_left, dirs_right) = compute_sss_dirs(R, tilt_angle, beam_width, 300);
    auto stop = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::microseconds>(stop - start);
    cout << "compute_sss_dirs time: " << duration.count() << " microseconds" << endl;

    start = chrono::high_resolution_clock::now();
    Eigen::MatrixXd hits_left(dirs_left.rows(), 3);
    Eigen::MatrixXd hits_right(dirs_right.rows(), 3);
    Eigen::VectorXi hits_left_inds(dirs_left.rows());
    Eigen::VectorXi hits_right_inds(dirs_right.rows());
    Eigen::VectorXd mod_left(dirs_left.rows());
    Eigen::VectorXd mod_right(dirs_right.rows());

    int nbr_lines = dirs_left.rows() + dirs_right.rows();
    Eigen::MatrixXd dirs = Eigen::MatrixXd(nbr_lines, dirs_left.cols());
    dirs << dirs_left, dirs_right;
    Eigen::MatrixXd P = Eigen::MatrixXd(nbr_lines, 3);
    P.rowwise() = origin.transpose();
    Eigen::MatrixXd hits = igl::embree::line_mesh_intersection(P, dirs, V1, F1);
    stop = chrono::high_resolution_clock::now();
    duration = chrono::duration_cast<chrono::microseconds>(stop - start);
    cout << "line_mesh_intersection time: " << duration.count() << " microseconds" << endl;

    start = chrono::high_resolution_clock::now();
    Eigen::MatrixXd global_hits = igl::barycentric_to_global(V1, F1, hits);
    stop = chrono::high_resolution_clock::now();
    duration = chrono::duration_cast<chrono::microseconds>(stop - start);
    cout << "barycentric_to_global time: " << duration.count() << " microseconds" << endl;

    start = chrono::high_resolution_clock::now();
    int hit_count = 0;
    for (int i = 0; i < dirs_left.rows(); ++i) {
        int hit = hits(i, 0);
        if (hit > 0) {
            int vind = F1(hit, 0);
            hits_left_inds(hit_count) = hit;
            // we actually get the coordinates within the triangle also, we could use that
            //hits_left.row(hit_count) = V1.row(vind);
            hits_left.row(hit_count) = global_hits.row(i);
            double nn = (origin - hits_left.row(hit_count).transpose()).norm();
            Eigen::Vector3d dir = origin - hits_left.row(hit_count).transpose();
            dir.normalize();
            mod_left(hit_count) = 1.; //(1./dir.dot(N_faces.row(hit).transpose()))*(nn/60.);
            ++hit_count;
        }
    }
    hits_left.conservativeResize(hit_count, 3);
    hits_left_inds.conservativeResize(hit_count);
    stop = chrono::high_resolution_clock::now();
    duration = chrono::duration_cast<chrono::microseconds>(stop - start);
    cout << "hits_left loop time: " << duration.count() << " microseconds" << endl;

    start = chrono::high_resolution_clock::now();
    hit_count = 0;
    for (int i = 0; i < dirs_right.rows(); ++i) {
        int hit = hits(dirs_left.rows() + i, 0);
        if (hit > 0) {
            int vind = F1(hit, 0);
            hits_right_inds(hit_count) = hit;
            // we actually get the coordinates within the triangle also, we could use that
            //hits_right.row(hit_count) = V1.row(vind);
            hits_right.row(hit_count) = global_hits.row(dirs_left.rows() + i);
            double nn = (origin - hits_right.row(hit_count).transpose()).norm();
            Eigen::Vector3d dir = origin - hits_right.row(hit_count).transpose();
            dir.normalize();
            mod_right(hit_count) = 1.; //(1./dir.dot(N_faces.row(hit).transpose()))*(nn/60.);
            ++hit_count;
        }
    }
    hits_right.conservativeResize(hit_count, 3);
    hits_right_inds.conservativeResize(hit_count);
    stop = chrono::high_resolution_clock::now();
    duration = chrono::duration_cast<chrono::microseconds>(stop - start);
    cout << "hits_right loop time: " << duration.count() << " microseconds" << endl;

    return make_tuple(hits_left, hits_right, hits_left_inds, hits_right_inds, mod_left, mod_right);
}

pair<Eigen::MatrixXd, Eigen::VectorXi> correlate_hits(const Eigen::MatrixXd& hits_port,
                                       const Eigen::VectorXi& hits_port_inds,
                                       const Eigen::VectorXd& mod_port,
                                       const xtf_sss_ping_side& ping,
                                       const Eigen::Vector3d& origin,
                                       double sound_vel,
                                       const Eigen::MatrixXi& F1,
                                       const csv_asvp_sound_speed::EntriesT& sound_speeds,
                                       bool sound_speed_layers,
                                       Eigen::MatrixXd& C,
                                       Eigen::VectorXd& hit_sums,
                                       Eigen::VectorXi& hit_counts,
                                       bool is_left)
{

    /*
    Eigen::VectorXd layer_depths(4);
    layer_depths << -5., -10., -15., -20.;
    Eigen::VectorXd layer_speeds(5);
    layer_speeds << 1506.43, 1504.47, 1498.61, 1495.05, 1492.64;
    */
    
    //bool sound_speed_layers = false; //!sound_speeds.empty();
    if (!sound_speeds.empty()) {
        sound_vel = sound_speeds[0].vels.head(sound_speeds[0].vels.rows()-1).mean();
    }

    // we do not take roll into account but we do need to account for the pitch
    // if the vehicle has pitch, we need to extend the layer depths
    // ok, let's just take the norm of the first two coordinates to begin with
    // actually, that already fixes the pitch rotation
    
    //Eigen::Vector3d origin = ping.pos_ - offset;
    Eigen::VectorXd times_port_simple = 2.*(hits_port.rowwise() - origin.transpose()).rowwise().norm()/sound_vel; //ping.sound_vel_;

    Eigen::VectorXd times_port;
    if (sound_speed_layers) {
        Eigen::VectorXd layer_depths = -sound_speeds[0].dbars.segment(1, sound_speeds[0].dbars.rows()-2);
        Eigen::VectorXd layer_speeds = sound_speeds[0].vels.head(sound_speeds[0].vels.rows()-1);

        Eigen::VectorXd x = (hits_port.leftCols<2>().rowwise() - origin.head<2>().transpose()).rowwise().norm();
        Eigen::MatrixXd end_points(x.rows(), 2);
        end_points.col(0) = x;
        end_points.col(1) = hits_port.col(2);
    
        Eigen::MatrixXd layer_widths;
        tie(times_port, layer_widths) = trace_multiple_layers(layer_depths, layer_speeds, end_points);
        times_port.array() *= 2.; // back and forth
        cout << "Got final times: " << times_port.transpose() << endl;

        visualize_rays(end_points, layer_depths, layer_widths, -25., false, is_left);
    }
    else {
        times_port = times_port_simple;
    }

    cout << "Compared to simple: " << times_port_simple.transpose() << endl;
    //Eigen::VectorXd times_stbd = 1.*(hits_stbd.rowwise() - origin.transpose()).rowwise().norm()/ping.sound_vel_;
    cout << "Port ping duration: " << ping.time_duration << endl;
    //cout << "Port times: " << times_port.transpose() << endl;
    //cout << "Stbd ping duration: " << ping.stbd.time_duration << endl;
    //cout << "Stbd times: " << times_stbd.transpose() << endl;
    

    if (times_port.rows() == 0) {
        cout << "There were no times computed!" << endl;
        return make_pair(Eigen::MatrixXd(0, 4), Eigen::VectorXi(0));
    }

    Eigen::MatrixXd hits_intensities(ping.pings.size(), 4);
    Eigen::VectorXi hits_pings_indices(ping.pings.size());

    double port_step = ping.time_duration / double(ping.pings.size());
    cout << "port step: " << port_step << endl;
    int pos = 0;
    int match_counter = 0;
    for (int i = 0; i < ping.pings.size(); ++i) {
        if (times_port(0) > double(i)*port_step) {
            continue;
        }
        if (pos >= hits_port_inds.rows()) {
            break;
        }

        //double intensity = (double(ping.port.pings[i]) + 32767.)/(2.*32767.);
        //double intensity = double(ping.pings[ping.pings.size()-i-1])/(10000.);
        /*if (intensity < 0.2) { // no hit?
            continue;
        }*/

        //cout << "Pos: " << pos << ", size: " << hits_port.rows() << endl;
        while (pos < hits_port.rows() && double(i)*port_step > times_port(pos)) {
            //cout << "Not good: " << double(i)*port_step << ", " << times_port(pos) << endl;
            ++pos;
        }
        if (pos >= hits_port.rows()) {
            break;
        }
        double intensity = mod_port(pos)*double(ping.pings[i])/(10000.);
        hits_intensities.row(match_counter).head<3>() = hits_port.row(pos);
        hits_intensities(match_counter, 3) = double(ping.pings[i])/10000.;
        hits_pings_indices(match_counter) = i;

        //cout << "Found one: " << double(i)*port_step << ", " << times_port(pos) << endl;
        //cout << "With intensity: " << intensity << endl;
        int vind = F1(hits_port_inds(pos), 0);
        hit_sums(vind) += intensity;
        hit_counts(vind) += 1;
        Eigen::Vector3d color = hit_sums(vind)/double(hit_counts(vind))*Eigen::Vector3d::Ones();
        C.row(vind) = color.transpose();
        ++match_counter;
    }
    hits_intensities.conservativeResize(match_counter, 4);
    hits_pings_indices.conservativeResize(match_counter);

    return make_pair(hits_intensities, hits_pings_indices);
}

bool point_in_view(const xtf_sss_ping& ping, const Eigen::Vector3d& point, double sensor_yaw)
{
    //Eigen::Matrix3d Ry = Eigen::AngleAxisd(ping.pitch_, Eigen::Vector3d::UnitY()).matrix();
    Eigen::Matrix3d Rcomp = Eigen::AngleAxisd(sensor_yaw, Eigen::Vector3d::UnitZ()).matrix();
    Eigen::Matrix3d Rz = Eigen::AngleAxisd(ping.heading_, Eigen::Vector3d::UnitZ()).matrix();
    Eigen::Matrix3d R = Rz*Rcomp; //*Ry;

    // first, let's transform the point to a coordinate system defined by the sonar
    Eigen::Vector3d p = R.transpose()*(point - ping.pos_);

    // now, let's get the yaw and pitch components
    double yaw = atan2(p(1), p(0));

    double xy_dist = fabs(p(1)); //sqrt(p(1)*p(1)+p(0)*p(0));
    double pitch = atan(p(2)/xy_dist);

    double min_pitch = -1.4*ping.port.tilt_angle - 0.5*ping.port.beam_width;
    double max_pitch = -1.4*ping.port.tilt_angle + 0.5*ping.port.beam_width - M_PI/20.;

    // check if point is in view of either of the side scans
    //bool yaw_in_view = fabs(yaw) < M_PI/2. + M_PI/16. && fabs(yaw) > M_PI/2. - M_PI/16.;
    bool yaw_in_view = fabs(p(0)) < 5.;

    bool pitch_in_view = pitch < max_pitch && pitch > min_pitch;

    cout << "Pitch: " << pitch << ", min pitch: " << min_pitch << ", max pitch: " << max_pitch << endl;

    cout << "Pitch in view?: " << pitch_in_view << " and yaw in view?: " << yaw_in_view << endl;

    cout << "XTF pos height: " << ping.pos_(2) << endl;
    cout << "Clicked oint height: " << point(2) << endl;
    cout << "Pos height: " << p(2) << endl;

    return pitch_in_view && yaw_in_view;
}
