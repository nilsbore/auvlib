#include <bathy_maps/draping_generator.h>

#include <igl/readSTL.h>
#include <bathy_maps/drape_mesh.h>

using namespace std;

draping_generator::draping_generator(const Eigen::MatrixXd& V1, const Eigen::MatrixXi& F1, const Eigen::MatrixXd& C1,
    const Eigen::MatrixXd& V2, const Eigen::MatrixXi& F2, const Eigen::MatrixXd& C2,
    const xtf_sss_ping::PingsT& pings, const Eigen::Vector3d& offset,
    const csv_asvp_sound_speed::EntriesT& sound_speeds)
    : pings(pings), i(0), V1(V1), F1(F1), V2(V2), F2(F2), offset(offset), sound_speeds(sound_speeds)
{
    //double first_heading = pings[0].heading_;
    hit_sums = Eigen::VectorXd(V1.rows()); hit_sums.setZero();
    hit_counts = Eigen::VectorXi(V1.rows()); hit_counts.setZero();

    V = Eigen::MatrixXd(V1.rows() + V2.rows(), V1.cols());
    V << V1, this->V2;
    F = Eigen::MatrixXi(F1.rows() + F2.rows(), F1.cols());
    F << F1, (F2.array() + V1.rows());

    V.bottomRows(V2.rows()) = V2;
    Eigen::Matrix3d Rz = Eigen::AngleAxisd(pings[0].heading_, Eigen::Vector3d::UnitZ()).matrix();
    V.bottomRows(V2.rows()) *= Rz.transpose();
    V.bottomRows(V2.rows()).array().rowwise() += (pings[0].pos_ - offset).transpose().array();

    C = Eigen::MatrixXd(C1.rows()+C2.rows(), C1.cols());
    C << C1, C2;


    // Initialize viewer

    // Compute per-face normals
    igl::per_face_normals(V1, F1, N_faces);

    viewer.data().set_mesh(V, F);
    // Add per-vertex colors
    //viewer.data().set_colors(C);
    // Add per-vertex colors
    viewer.data().set_colors(C);

    viewer.data().point_size = 10;
    viewer.data().line_width = 1;

    viewer.callback_pre_draw = std::bind(&draping_generator::callback_pre_draw, this, std::placeholders::_1);

    viewer.core.is_animating = true;
    viewer.core.animation_max_fps = 30.;
    //viewer.launch();
    viewer.core.background_color << 1., 1., 1., 1.; // white background
}

void draping_generator::launch()
{
    viewer.launch();
}

tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::Vector3d> draping_generator::project_sss()
{

    cout << "Setting new position: " << pings[i].pos_.transpose() << endl;
    //viewer.data().compute_normals();
    Eigen::Matrix3d Rcomp = Eigen::AngleAxisd(1.*5.*M_PI/180., Eigen::Vector3d::UnitZ()).matrix();
    Eigen::Matrix3d Ry = Eigen::AngleAxisd(pings[i].pitch_, Eigen::Vector3d::UnitY()).matrix();
    Eigen::Matrix3d Rz = Eigen::AngleAxisd(pings[i].heading_, Eigen::Vector3d::UnitZ()).matrix();
    Eigen::Matrix3d R = Rz*Ry*Rcomp ;

    Eigen::MatrixXd hits_left;
    Eigen::MatrixXd hits_right;
    Eigen::VectorXi hits_left_inds;
    Eigen::VectorXi hits_right_inds;
    Eigen::VectorXd mod_left;
    Eigen::VectorXd mod_right;
    //tie(hits_left, hits_right, hits_left_inds, hits_right_inds) = compute_hits(pings[i].pos_ - offset, R, pings[i].port.tilt_angle, pings[i].port.beam_width, V1, F1);
    auto start = chrono::high_resolution_clock::now();
    tie(hits_left, hits_right, hits_left_inds, hits_right_inds, mod_left, mod_right) = embree_compute_hits(pings[i].pos_ - offset, R, 1.4*pings[i].port.tilt_angle, pings[i].port.beam_width, V1, F1);
    auto stop = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::microseconds>(stop - start);
    cout << "embree_compute_hits time: " << duration.count() << " microseconds" << endl;

    start = chrono::high_resolution_clock::now();
    Eigen::MatrixXd hits_left_intensities = correlate_hits(hits_left, hits_left_inds, mod_left, pings[i].port, pings[i].pos_ - offset, pings[i].sound_vel_, F1, sound_speeds, C, hit_sums, hit_counts);
    stop = chrono::high_resolution_clock::now();
    duration = chrono::duration_cast<chrono::microseconds>(stop - start);
    cout << "left correlate_hits time: " << duration.count() << " microseconds" << endl;

    start = chrono::high_resolution_clock::now();
    Eigen::MatrixXd hits_right_intensities = correlate_hits(hits_right, hits_right_inds, mod_right, pings[i].stbd, pings[i].pos_ - offset, pings[i].sound_vel_, F1, sound_speeds, C, hit_sums, hit_counts);
    stop = chrono::high_resolution_clock::now();
    duration = chrono::duration_cast<chrono::microseconds>(stop - start);
    cout << "right correlate_hits time: " << duration.count() << " microseconds" << endl;

    if (i % 10 == 0) {
        start = chrono::high_resolution_clock::now();
        V.bottomRows(V2.rows()) = V2;
        V.bottomRows(V2.rows()) *= R.transpose();
        V.bottomRows(V2.rows()).array().rowwise() += (pings[i].pos_ - offset).transpose().array();
        viewer.data().set_vertices(V);

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

    return make_tuple(hits_left_intensities, hits_right_intensities, pings[i].pos_ - offset);
}


bool draping_generator::callback_pre_draw(igl::opengl::glfw::Viewer& viewer)
{
    glEnable(GL_CULL_FACE);

    if (viewer.core.is_animating && i < pings.size()) {
        project_sss();
        ++i;
    }

    return false;
}

void generate_draping(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F,
                      const draping_generator::BoundsT& bounds, const xtf_sss_ping::PingsT& pings,
                      const csv_asvp_sound_speed::EntriesT& sound_speeds)
{
    Eigen::MatrixXd C_jet;
    igl::jet(V.col(2), true, C_jet);

    Eigen::MatrixXd Vb;
    Eigen::MatrixXi Fb;
    Eigen::MatrixXd Nb;
    igl::readSTL("5TUM.stl", Vb, Fb, Nb);
    Eigen::MatrixXd Cb(Vb.rows(), 3);
    Cb.rowwise() = Eigen::RowVector3d(1., 1., 0.);
    Vb.array() *= 0.01;
    Eigen::Matrix3d Rz = Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitZ()).matrix();
    Vb *= Rz.transpose();
    //display_mesh(Vb, Fb);

    Eigen::Vector3d offset(bounds(0, 0), bounds(0, 1), 0.);
    draping_generator viewer(V, F, C_jet, Vb, Fb, Cb, pings, offset, sound_speeds);
    viewer.launch();

}
