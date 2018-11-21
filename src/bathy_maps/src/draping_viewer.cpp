#include <bathy_maps/draping_viewer.h>

#include <igl/readSTL.h>
#include <bathy_maps/drape_mesh.h>

using namespace std;

survey_viewer::survey_viewer(const Eigen::MatrixXd& V1, const Eigen::MatrixXi& F1, const Eigen::MatrixXd& C1,
    const Eigen::MatrixXd& V2, const Eigen::MatrixXi& F2, const Eigen::MatrixXd& C2,
    const xtf_sss_ping::PingsT& pings, const Eigen::Vector3d& offset)
    : pings(pings), i(0), V1(V1), F1(F1), V2(V2), F2(F2), offset(offset)
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

    is_active = Eigen::VectorXi(pings.size()); is_active.setOnes();

    // Compute per-face normals
    igl::per_face_normals(V1, F1, N_faces);

    viewer.data().set_mesh(V, F);
    // Add per-vertex colors
    //viewer.data().set_colors(C);
    // Add per-vertex colors
    viewer.data().set_colors(C);

    viewer.data().point_size = 10;
    viewer.data().line_width = 1;

    //viewer.callback_pre_draw = std::bind(&survey_viewer::callback_pre_draw, this, std::placeholders::_1);
    viewer.callback_mouse_down = std::bind(&survey_viewer::callback_mouse_down, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    viewer.callback_key_pressed = std::bind(&survey_viewer::callback_key_pressed, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
    viewer.core.is_animating = true;
    viewer.core.animation_max_fps = 30.;
    //viewer.launch();
    viewer.core.background_color << 1., 1., 1., 1.; // white background
}

void survey_viewer::launch()
{
    viewer.launch();
}


void survey_viewer::project_sss()
{
    for (; i < pings.size() && is_active[i] == 0; ++i) {}

    if (i >= pings.size()) {
        return;
    }

    cout << "Setting new position: " << pings[i].pos_.transpose() << endl;
    //viewer.data().compute_normals();
    Eigen::Matrix3d Ry = Eigen::AngleAxisd(pings[i].pitch_, Eigen::Vector3d::UnitY()).matrix();
    Eigen::Matrix3d Rz = Eigen::AngleAxisd(pings[i].heading_, Eigen::Vector3d::UnitZ()).matrix();
    Eigen::Matrix3d R = Rz*Ry;

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
    correlate_hits(hits_left, hits_left_inds, mod_left, pings[i].port, pings[i].pos_ - offset, pings[i].sound_vel_, F1, C, hit_sums, hit_counts);
    stop = chrono::high_resolution_clock::now();
    duration = chrono::duration_cast<chrono::microseconds>(stop - start);
    cout << "left correlate_hits time: " << duration.count() << " microseconds" << endl;

    start = chrono::high_resolution_clock::now();
    correlate_hits(hits_right, hits_right_inds, mod_right, pings[i].stbd, pings[i].pos_ - offset, pings[i].sound_vel_, F1, C, hit_sums, hit_counts);
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

}


bool survey_viewer::callback_pre_draw(igl::opengl::glfw::Viewer& viewer)
{
    glEnable(GL_CULL_FACE);

    if (viewer.core.is_animating) {
        project_sss();
    }
}

bool survey_viewer::callback_mouse_down(igl::opengl::glfw::Viewer& viewer, int, int)
{
    cout << "Got mouse callback!" << endl;
    int fid;
    Eigen::Vector3f bc;
    // Cast a ray in the view direction starting from the mouse position
    double x = viewer.current_mouse_x;
    double y = viewer.core.viewport(3) - viewer.current_mouse_y;
    if (igl::unproject_onto_mesh(Eigen::Vector2f(x, y), viewer.core.view * viewer.core.model,
            viewer.core.proj, viewer.core.viewport, V1, F1, fid, bc)) {
        // paint hit red
        cout << "Got point in mesh!" << endl;
        int vind = F1(fid, 0);
        C.row(vind) << 1, 0, 0;
        viewer.data().set_colors(C);
        Eigen::Vector3d point = offset + V1.row(vind).transpose();
        int nbr_in_view = 0;
        for (int j = 0; j < pings.size(); ++j) {
            is_active(j) = int(point_in_view(pings[j], point));
            nbr_in_view += is_active(j);
        }
        i = 0;

        cout << "Number in view: " << nbr_in_view << " out of: " << pings.size() << endl;

        return true;
    }
    cout << "Not in mesh!" << endl;
    return false;
}

bool survey_viewer::callback_key_pressed(igl::opengl::glfw::Viewer& viewer, unsigned int key, int mods)
{
    switch (key) {
    case 'n':
        project_sss();
        i += 1;
        return true;
    case 'm':
        project_sss();
        i += 100;
        return true;
    default:
        return false;
    }
}

void overlay_sss(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F,
                 const survey_viewer::BoundsT& bounds, const xtf_sss_ping::PingsT& pings)
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
    survey_viewer viewer(V, F, C_jet, Vb, Fb, Cb, pings, offset);
    viewer.launch();
}
