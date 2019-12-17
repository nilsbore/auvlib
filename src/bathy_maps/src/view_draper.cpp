#include <bathy_maps/view_draper.h>

#include <igl/readSTL.h>

using namespace std;
using namespace xtf_data;
using namespace csv_data;

ViewDraper::ViewDraper(const Eigen::MatrixXd& V1, const Eigen::MatrixXi& F1,
                       const xtf_sss_ping::PingsT& pings,
                       const BoundsT& bounds,
                       const csv_asvp_sound_speed::EntriesT& sound_speeds)
    : BaseDraper(V1, F1, bounds, sound_speeds), pings(pings), i(0)
{
    V = V1;
    F = F1;
    C = color_jet_from_mesh(V1);

    V2 = Eigen::MatrixXd(0, V1.cols());
    F2 = Eigen::MatrixXi(0, F1.cols());

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

    viewer.callback_pre_draw = std::bind(&ViewDraper::callback_pre_draw, this, std::placeholders::_1);

    viewer.core().is_animating = true;
    viewer.core().animation_max_fps = 30.;
    //viewer.launch();
    viewer.core().background_color << 1., 1., 1., 1.; // white background

    //Eigen::Matrix2d mesh_bounds; mesh_bounds << 0., 0., bounds(1, 0)-bounds(0, 0), bounds(1, 1)-bounds(0, 1);
    set_texture(texture_image, bounds);
}

void ViewDraper::set_texture(const Eigen::MatrixXd& texture, const BoundsT& texture_bounds)
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

void ViewDraper::set_vehicle_mesh(const Eigen::MatrixXd& new_V2, const Eigen::MatrixXi& new_F2, const Eigen::MatrixXd& new_C2)
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

void ViewDraper::show()
{
    viewer.launch();
}

bool ViewDraper::callback_pre_draw(igl::opengl::glfw::Viewer& viewer)
{
    glEnable(GL_CULL_FACE);

    if (viewer.core().is_animating && i < pings.size()) {
        //project_sss();
        Eigen::MatrixXd hits_left;
        Eigen::MatrixXd hits_right;
        Eigen::MatrixXd normals_left;
        Eigen::MatrixXd normals_right;
        tie(hits_left, hits_right, normals_left, normals_right) = project(pings[i]);

        Eigen::Vector3d origin_port;
        Eigen::Vector3d origin_stbd;
        tie(origin_port, origin_stbd) = get_port_stbd_sensor_origins(pings[i]);
        Eigen::VectorXd times_left = compute_times(origin_port, hits_left);
        Eigen::VectorXd times_right = compute_times(origin_stbd, hits_right);

        if (i % 10 == 0) {
            visualize_vehicle();
            //visualize_rays(hits_left, hits_right);
            visualize_rays(origin_port, hits_left, true);
            visualize_rays(origin_stbd, hits_right);
        }
        ++i;
    }

    return false;
}

Eigen::MatrixXd color_jet_from_mesh(const Eigen::MatrixXd& V)
{
    Eigen::MatrixXd C_jet;
    igl::jet(V.col(2), true, C_jet);
    return C_jet;
}

void drape_viewer(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F,
                  const ViewDraper::BoundsT& bounds, const xtf_sss_ping::PingsT& pings,
                  const csv_asvp_sound_speed::EntriesT& sound_speeds, double sensor_yaw)
{
    Eigen::MatrixXd Vb;
    Eigen::MatrixXi Fb;
    Eigen::MatrixXd Cb;
    tie(Vb, Fb, Cb) = get_vehicle_mesh();

    ViewDraper viewer(V, F, pings, bounds, sound_speeds);
    viewer.set_sidescan_yaw(sensor_yaw);
    viewer.set_vehicle_mesh(Vb, Fb, Cb);
    //viewer.set_ray_tracing_enabled(true);
    viewer.show();
}

void ViewDraper::visualize_rays(const Eigen::Vector3d& sensor_origin, const Eigen::MatrixXd& hits, bool clear)
{
    Eigen::MatrixXi E;
    Eigen::MatrixXd P(hits.rows(), 3);
    P.rowwise() = sensor_origin.transpose();
    if (clear) {
        viewer.data().set_edges(P, E, Eigen::RowVector3d(1., 0., 0.));
        viewer.data().add_edges(P, hits, Eigen::RowVector3d(1., 0., 0.));
    }
    else {
        viewer.data().add_edges(P, hits, Eigen::RowVector3d(0., 1., 0.));
    }
}

/*
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
*/

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

void ViewDraper::visualize_vehicle()
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

void ViewDraper::visualize_intensities()
{
    set_texture(texture_image, bounds);
}
