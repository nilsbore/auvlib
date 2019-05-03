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

#include <igl/readSTL.h>
#include <bathy_maps/drape_mesh.h>
#include <bathy_maps/mesh_map.h>

using namespace std;
using namespace xtf_data;
using namespace csv_data;

BaseDraper::BaseDraper(const Eigen::MatrixXd& V1, const Eigen::MatrixXi& F1,
                       const xtf_sss_ping::PingsT& pings,
                       const BoundsT& bounds,
                       const csv_asvp_sound_speed::EntriesT& sound_speeds)
    : pings(pings), i(0), V1(V1), F1(F1),
      sound_speeds(sound_speeds), bounds(bounds),
      sensor_yaw(0.), ray_tracing_enabled(false)
{
    offset = Eigen::Vector3d(bounds(0, 0), bounds(0, 1), 0.);

    //double first_heading = pings[0].heading_;
    //hit_sums = Eigen::VectorXd(V1.rows()); hit_sums.setZero();
    //hit_counts = Eigen::VectorXi(V1.rows()); hit_counts.setZero();

    V = V1;
    F = F1;
    C = color_jet_from_mesh(V1);

    V2 = Eigen::MatrixXd(0, V1.cols());
    F2 = Eigen::MatrixXi(0, F1.cols());

    pos_small = -1000.*Eigen::Vector3d::Ones(); // should be outside area

    // resolution of 1m
    int rows = int(bounds(1, 1)-bounds(0, 1));
    int cols = int(bounds(1, 0)-bounds(0, 0));
    texture_image = Eigen::MatrixXd::Zero(rows, cols);
    cout << "Texture image rows: " << rows << ", cols: " << cols << endl;
    for (int j = 0; j < V1.rows(); ++j) {
        //int y = rows-int(V1(j, 1))-1;
        int y = int(V1(j, 1));
        int x = int(V1(j, 0));
        if (x >= 0 && x < cols && y >= 0 && y < rows && V1(j, 2) != 0.) {
            texture_image(y, x) = 1.; //V1(j, 0)/(bounds(1, 0)-bounds(0, 0));
        }
    }

    // Initialize viewer

    // Compute per-face normals
    //igl::per_face_normals(V1, F1, N_faces);

    viewer.data().set_mesh(V, F);
    // Add per-vertex colors
    //viewer.data().set_colors(C);
    // Add per-vertex colors
    viewer.data().set_colors(C);

    viewer.data().point_size = 10;
    viewer.data().line_width = 1;

    viewer.callback_pre_draw = std::bind(&BaseDraper::callback_pre_draw, this, std::placeholders::_1);

    viewer.core.is_animating = true;
    viewer.core.animation_max_fps = 30.;
    //viewer.launch();
    viewer.core.background_color << 1., 1., 1., 1.; // white background

    //Eigen::Matrix2d mesh_bounds; mesh_bounds << 0., 0., bounds(1, 0)-bounds(0, 0), bounds(1, 1)-bounds(0, 1);
    set_texture(texture_image, bounds);
}

bool BaseDraper::fast_is_mesh_underneath_vehicle(const Eigen::Vector3d& origin)
{
    //int y = texture_image.rows()-int(origin(1))-1;
    int y = int(origin(1));
    int x = int(origin(0));
    if (x >= 0 && x < texture_image.cols() && y >= 0 && y < texture_image.rows()) {
        return texture_image(y, x) != 0.;
    }
    return false;
}

void BaseDraper::add_texture_intensities(const Eigen::MatrixXd& hits, const Eigen::VectorXd& intensities)
{
    for (int j = 0; j < hits.rows(); ++j) {
        int y = int(hits(j, 1));
        int x = int(hits(j, 0));
        if (x >= 0 && x < texture_image.cols() && y >= 0 && y < texture_image.rows()) {
            texture_image(y, x) = std::max(intensities(j), 0.01);
        }
    }
}

void BaseDraper::set_texture(const Eigen::MatrixXd& texture, const BoundsT& texture_bounds)
{
    Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic> R = (255.*texture.transpose()).cast<uint8_t>();
    Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic> G = (255.*texture.transpose()).cast<uint8_t>();
    Eigen::Matrix<unsigned char,Eigen::Dynamic,Eigen::Dynamic> B = (255.*texture.transpose()).cast<uint8_t>();

    // TODO: fix, this assumes that we are always starting at (0, 0)
    Eigen::MatrixXd UV = V.leftCols<2>();
    //UV.col(0).array() -= bounds(0, 0);
    UV.col(0).array() /= bounds(1, 0) - bounds(0, 0);
    //UV.col(1).array() -= bounds(0, 1);
    UV.col(1).array() /= bounds(1, 1) - bounds(0, 1);

    viewer.data().set_uv(UV);
    viewer.data().show_texture = true;
    // Use the image as a texture
    viewer.data().set_texture(R, G, B);
}

void BaseDraper::set_vehicle_mesh(const Eigen::MatrixXd& new_V2, const Eigen::MatrixXi& new_F2, const Eigen::MatrixXd& new_C2)
{
    V2 = new_V2;
    F2 = new_F2;
    Eigen::MatrixXd C2 = new_C2;

    V.conservativeResize(V1.rows() + V2.rows(), V1.cols());
    V.bottomRows(V2.rows()) = V2;
    F.conservativeResize(F1.rows() + F2.rows(), F1.cols());
    F.bottomRows(F2.rows()) = F2.array() + V1.rows();

    Eigen::Matrix3d Rz = Eigen::AngleAxisd(pings[0].heading_, Eigen::Vector3d::UnitZ()).matrix();
    V.bottomRows(V2.rows()) *= Rz.transpose();
    V.bottomRows(V2.rows()).array().rowwise() += (pings[0].pos_ - offset).transpose().array();

    C.conservativeResize(V1.rows()+C2.rows(), C.cols());
    C.bottomRows(C2.rows()) = C2;

    viewer.data().clear();
    viewer.data().set_mesh(V, F);
    viewer.data().set_colors(C);

    set_texture(texture_image, bounds);
}

void BaseDraper::show()
{
    viewer.launch();
}

/*
tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::VectorXi, Eigen::VectorXi, Eigen::Vector3d> BaseDraper::project_sss()
{

    cout << "Setting new position: " << pings[i].pos_.transpose() << endl;
    //viewer.data().compute_normals();
    Eigen::Matrix3d Rcomp = Eigen::AngleAxisd(sensor_yaw, Eigen::Vector3d::UnitZ()).matrix();
    Eigen::Matrix3d Ry = Eigen::AngleAxisd(pings[i].pitch_, Eigen::Vector3d::UnitY()).matrix();
    Eigen::Matrix3d Rz = Eigen::AngleAxisd(pings[i].heading_, Eigen::Vector3d::UnitZ()).matrix();
    Eigen::Matrix3d R = Rz*Ry*Rcomp;

    Eigen::MatrixXd hits_left;
    Eigen::MatrixXd hits_right;
    Eigen::VectorXi hits_left_inds;
    Eigen::VectorXi hits_right_inds;
    Eigen::VectorXd mod_left;
    Eigen::VectorXd mod_right;
    //tie(hits_left, hits_right, hits_left_inds, hits_right_inds) = compute_hits(pings[i].pos_ - offset, R, pings[i].port.tilt_angle, pings[i].port.beam_width, V1, F1);
    auto start = chrono::high_resolution_clock::now();
    tie(hits_left, hits_right, hits_left_inds, hits_right_inds, mod_left, mod_right) = embree_compute_hits(pings[i].pos_ - offset, R, 1.4*pings[i].port.tilt_angle, pings[i].port.beam_width + 0.2, V1, F1);
    auto stop = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::microseconds>(stop - start);
    cout << "embree_compute_hits time: " << duration.count() << " microseconds" << endl;

    start = chrono::high_resolution_clock::now();
    Eigen::MatrixXd hits_left_intensities;
    Eigen::VectorXi hits_left_pings_indices;
    tie(hits_left_intensities, hits_left_pings_indices) = correlate_hits(hits_left, hits_left_inds, mod_left, pings[i].port, pings[i].pos_ - offset, pings[i].sound_vel_, F1, sound_speeds, ray_tracing_enabled, C, hit_sums, hit_counts, true);
    stop = chrono::high_resolution_clock::now();
    duration = chrono::duration_cast<chrono::microseconds>(stop - start);
    cout << "left correlate_hits time: " << duration.count() << " microseconds" << endl;

    start = chrono::high_resolution_clock::now();
    Eigen::MatrixXd hits_right_intensities;
    Eigen::VectorXi hits_right_pings_indices;
    tie(hits_right_intensities, hits_right_pings_indices)= correlate_hits(hits_right, hits_right_inds, mod_right, pings[i].stbd, pings[i].pos_ - offset, pings[i].sound_vel_, F1, sound_speeds, ray_tracing_enabled, C, hit_sums, hit_counts, false);
    stop = chrono::high_resolution_clock::now();
    duration = chrono::duration_cast<chrono::microseconds>(stop - start);
    cout << "right correlate_hits time: " << duration.count() << " microseconds" << endl;

    if (i % 10 == 0) {
        start = chrono::high_resolution_clock::now();
        if (V2.rows() > 0) {
            V.bottomRows(V2.rows()) = V2;
            V.bottomRows(V2.rows()) *= R.transpose();
            V.bottomRows(V2.rows()).array().rowwise() += (pings[i].pos_ - offset).transpose().array();
            viewer.data().set_vertices(V);
        }

        Eigen::MatrixXi E;
        Eigen::MatrixXd P(hits_left.rows(), 3);
        P.rowwise() = (pings[i].pos_ - offset).transpose();
        viewer.data().set_edges(P, E, Eigen::RowVector3d(1., 0., 0.));
        viewer.data().add_edges(P, hits_left, Eigen::RowVector3d(1., 0., 0.));
        P = Eigen::MatrixXd(hits_right.rows(), 3);
        P.rowwise() = (pings[i].pos_ - offset).transpose();
        viewer.data().add_edges(P, hits_right, Eigen::RowVector3d(0., 1., 0.));
        viewer.data().set_colors(C);
        stop = chrono::high_resolution_clock::now();
        duration = chrono::duration_cast<chrono::microseconds>(stop - start);
        cout << "vis time: " << duration.count() << " microseconds" << endl;
    }

    return make_tuple(hits_left_intensities, hits_right_intensities, hits_left_pings_indices, hits_right_pings_indices, pings[i].pos_ - offset);
}
*/

bool BaseDraper::callback_pre_draw(igl::opengl::glfw::Viewer& viewer)
{
    glEnable(GL_CULL_FACE);

    if (viewer.core.is_animating && i < pings.size()) {
        //project_sss();
        Eigen::MatrixXd hits_left;
        Eigen::MatrixXd hits_right;
        Eigen::MatrixXd normals_left;
        Eigen::MatrixXd normals_right;
        tie(hits_left, hits_right, normals_left, normals_right) = project();

        if (i % 10 == 0) {
            visualize_vehicle();
            visualize_rays(hits_left, hits_right);
        }
        ++i;
    }

    return false;
}

void BaseDraper::set_ray_tracing_enabled(bool enabled)
{
    ray_tracing_enabled = enabled;
}

tuple<Eigen::MatrixXd, Eigen::MatrixXi, Eigen::MatrixXd> get_vehicle_mesh()
{
    Eigen::MatrixXd Vb;
    Eigen::MatrixXi Fb;
    Eigen::MatrixXd Nb;
    igl::readSTL("5TUM.stl", Vb, Fb, Nb);
    Eigen::MatrixXd Cb(Vb.rows(), 3);
    Cb.rowwise() = Eigen::RowVector3d(1., 1., 0.);
    Vb.array() *= 0.01;
    Eigen::Matrix3d Rz = Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitZ()).matrix();
    Vb *= Rz.transpose();

    return make_tuple(Vb, Fb, Cb);
}

Eigen::MatrixXd color_jet_from_mesh(const Eigen::MatrixXd& V)
{
    Eigen::MatrixXd C_jet;
    igl::jet(V.col(2), true, C_jet);
    return C_jet;
}

void drape_viewer(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F,
                  const BaseDraper::BoundsT& bounds, const xtf_sss_ping::PingsT& pings,
                  const csv_asvp_sound_speed::EntriesT& sound_speeds, double sensor_yaw)
{
    Eigen::MatrixXd Vb;
    Eigen::MatrixXi Fb;
    Eigen::MatrixXd Cb;
    tie(Vb, Fb, Cb) = get_vehicle_mesh();

    BaseDraper viewer(V, F, pings, bounds, sound_speeds);
    viewer.set_sidescan_yaw(sensor_yaw);
    viewer.set_vehicle_mesh(Vb, Fb, Cb);
    //viewer.set_ray_tracing_enabled(true);
    viewer.show();
}

// New style functions follow here:
tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd> BaseDraper::project()
{
    cout << "Setting new position: " << pings[i].pos_.transpose() << endl;
    Eigen::Matrix3d Rcomp = Eigen::AngleAxisd(sensor_yaw, Eigen::Vector3d::UnitZ()).matrix();
    Eigen::Matrix3d Ry = Eigen::AngleAxisd(pings[i].pitch_, Eigen::Vector3d::UnitY()).matrix();
    Eigen::Matrix3d Rz = Eigen::AngleAxisd(pings[i].heading_, Eigen::Vector3d::UnitZ()).matrix();
    Eigen::Matrix3d R = Rz*Ry*Rcomp;

    Eigen::MatrixXd hits_left;
    Eigen::MatrixXd hits_right;
    Eigen::VectorXi hits_left_inds;
    Eigen::VectorXi hits_right_inds;
    Eigen::VectorXd mod_left;
    Eigen::VectorXd mod_right;

    Eigen::Vector3d offset_pos = pings[i].pos_ - offset;
    if ((offset_pos - pos_small).norm() > 50.) {
        tie(V1_small, F1_small) = mesh_map::cut_square_around_point(V1, F1, offset_pos.head<2>(), 200.);
        if (V1_small.rows() == 0) {
            V1_small = V1;
            F1_small = F1;
        }
        igl::per_face_normals(V1_small, F1_small, N_small); // TODO: compute N_small together with F1_small
        pos_small = offset_pos;
    }

    double beam_width;
    double tilt_angle;
    if (true) {
        double depth = .8*depth_mesh_underneath_vehicle(offset_pos, V1_small, F1_small); // make it slightly wider
        double max_distance = .5*compute_simple_sound_vel()*pings[i].port.time_duration;
        beam_width = acos(depth/max_distance);
        tilt_angle = beam_width/2.;
    }
    else {
        beam_width = pings[i].port.beam_width + 0.2;
        tilt_angle = 1.4*pings[i].port.tilt_angle;
    }

    auto start = chrono::high_resolution_clock::now();
    //tie(hits_left, hits_right, hits_left_inds, hits_right_inds, mod_left, mod_right) = embree_compute_hits(offset_pos, R, 1.4*pings[i].port.tilt_angle, pings[i].port.beam_width + 0.2, V1_small, F1_small);
    tie(hits_left, hits_right, hits_left_inds, hits_right_inds, mod_left, mod_right) = embree_compute_hits(offset_pos, R, tilt_angle, beam_width, V1_small, F1_small);
    auto stop = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::microseconds>(stop - start);
    cout << "embree_compute_hits time: " << duration.count() << " microseconds" << endl;

    Eigen::MatrixXd normals_left(hits_left.rows(), 3);
    Eigen::MatrixXd normals_right(hits_right.rows(), 3);

    for (int j = 0; j < hits_left.rows(); ++j) {
        normals_left.row(j) = N_small.row(hits_left_inds(j));
    }

    for (int j = 0; j < hits_right.rows(); ++j) {
        normals_right.row(j) = N_small.row(hits_right_inds(j));
    }

    return make_tuple(hits_left, hits_right, normals_left, normals_right);
}

double BaseDraper::compute_simple_sound_vel()
{
    double sound_vel = sound_speeds[0].vels.head(sound_speeds[0].vels.rows()-1).mean();
    return sound_vel;
}

Eigen::VectorXd BaseDraper::compute_times(const Eigen::MatrixXd& P)
{
    Eigen::Vector3d pos = pings[i].pos_ - offset;
    double sound_vel = compute_simple_sound_vel();
    Eigen::VectorXd times = 2.*(P.rowwise() - pos.transpose()).rowwise().norm()/sound_vel;
    return times;
}

Eigen::VectorXi BaseDraper::compute_bin_indices(const Eigen::VectorXd& times, const xtf_data::xtf_sss_ping_side& ping, size_t nbr_windows)
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
                                                 const xtf_data::xtf_sss_ping_side& ping, size_t nbr_windows)
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

Eigen::VectorXd BaseDraper::compute_intensities(const Eigen::VectorXd& times, 
                                                const xtf_data::xtf_sss_ping_side& ping)
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

void BaseDraper::visualize_rays(const Eigen::MatrixXd& hits_left, const Eigen::MatrixXd& hits_right)
{
    Eigen::MatrixXi E;
    Eigen::MatrixXd P(hits_left.rows(), 3);
    P.rowwise() = (pings[i].pos_ - offset).transpose();
    viewer.data().set_edges(P, E, Eigen::RowVector3d(1., 0., 0.));
    viewer.data().add_edges(P, hits_left, Eigen::RowVector3d(1., 0., 0.));
    P = Eigen::MatrixXd(hits_right.rows(), 3);
    P.rowwise() = (pings[i].pos_ - offset).transpose();
    viewer.data().add_edges(P, hits_right, Eigen::RowVector3d(0., 1., 0.));
}

void BaseDraper::visualize_vehicle()
{
    if (V2.rows() == 0) {
        return;
    }
    Eigen::Matrix3d Rcomp = Eigen::AngleAxisd(sensor_yaw, Eigen::Vector3d::UnitZ()).matrix();
    Eigen::Matrix3d Ry = Eigen::AngleAxisd(pings[i].pitch_, Eigen::Vector3d::UnitY()).matrix();
    Eigen::Matrix3d Rz = Eigen::AngleAxisd(pings[i].heading_, Eigen::Vector3d::UnitZ()).matrix();
    Eigen::Matrix3d R = Rz*Ry*Rcomp;

    if (true) {//V1_small.rows() == 0) {
        V.bottomRows(V2.rows()) = V2;
        V.bottomRows(V2.rows()) *= R.transpose();
        V.bottomRows(V2.rows()).array().rowwise() += (pings[i].pos_ - offset).transpose().array();
        viewer.data().set_vertices(V);
    }
    else {
        Eigen::MatrixXd V_new(V1_small.rows()+V2.rows(), V1_small.cols());
        V_new.topRows(V1_small.rows()) = V1_small;
        V_new.bottomRows(V2.rows()) = V2;
        V_new.bottomRows(V2.rows()) *= R.transpose();
        V_new.bottomRows(V2.rows()).array().rowwise() += (pings[i].pos_ - offset).transpose().array();
        Eigen::MatrixXi F_new(F1_small.rows()+F2.rows(), F1_small.cols());
        F_new.topRows(F1_small.rows()) = F1_small;
        F_new.bottomRows(F2.rows()) = F2.array() + V1_small.rows();
        Eigen::MatrixXd C_new(V_new.rows(), V_new.cols());
        C_new.topRows(V1_small.rows()) = color_jet_from_mesh(V1_small);
        C_new.bottomRows(V2.rows()).rowwise() = Eigen::RowVector3d(1., 1., 0.);
        viewer.data().clear();
        viewer.data().set_mesh(V_new, F_new);
        viewer.data().set_colors(C_new);
    }
}

void BaseDraper::visualize_intensities()
{
    set_texture(texture_image, bounds);
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
