#include <bathy_maps/view_draper.h>

#include <igl/readSTL.h>

using namespace std;
using namespace csv_data;

ViewDraper::ViewDraper(const Eigen::MatrixXd& V1, const Eigen::MatrixXi& F1,
                       const std_data::sss_ping::PingsT& pings,
                       const BoundsT& bounds,
                       const csv_asvp_sound_speed::EntriesT& sound_speeds)
    : BaseDraper(V1, F1, bounds, sound_speeds), pings(pings), i(0), nbr_time_bins(256)
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
    viewer.data().show_lines = false;

    viewer.callback_pre_draw = std::bind(&ViewDraper::callback_pre_draw, this, std::placeholders::_1);

    viewer.core().is_animating = true;
    viewer.core().animation_max_fps = 30.;
    //viewer.launch();
    viewer.core().background_color << 1., 1., 1., 1.; // white background

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

    //Eigen::Matrix2d mesh_bounds; mesh_bounds << 0., 0., bounds(1, 0)-bounds(0, 0), bounds(1, 1)-bounds(0, 1);
    set_texture(texture_image, bounds);
}

bool ViewDraper::fast_is_mesh_underneath_vehicle(const Eigen::Vector3d& origin)
{
    //int y = texture_image.rows()-int(origin(1))-1;
    int y = int(origin(1));
    int x = int(origin(0));
    if (x >= 0 && x < texture_image.cols() && y >= 0 && y < texture_image.rows()) {
        return texture_image(y, x) != 0.;
    }
    return false;
}

void ViewDraper::add_texture_intensities(const Eigen::MatrixXd& hits, const Eigen::VectorXd& intensities)
{
    for (int j = 0; j < hits.rows(); ++j) {
        int y = int(hits(j, 1));
        int x = int(hits(j, 0));
        if (intensities(j) != 0 && x >= 0 && x < texture_image.cols() && y >= 0 && y < texture_image.rows()) {
            texture_image(y, x) = intensity_multiplier*std::max(intensities(j), 0.01);
        }
    }
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
        ping_draping_result left, right;
        tie(left, right) = project_ping(pings[i], nbr_time_bins);

        add_texture_intensities(left.hits_points, left.hits_intensities);
        add_texture_intensities(right.hits_points, right.hits_intensities);

        if (save_callback) {
            save_callback(left, right);
        }

        if (i % 10 == 0) {
            Eigen::Vector3d origin_port;
            Eigen::Vector3d origin_stbd;
            tie(origin_port, origin_stbd) = get_port_stbd_sensor_origins(pings[i]);

            visualize_vehicle();
            visualize_intensities();
            visualize_rays(origin_port, left.hits_points, true);
            visualize_rays(origin_stbd, right.hits_points);
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
                  const ViewDraper::BoundsT& bounds, const std_data::sss_ping::PingsT& pings,
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

    V.bottomRows(V2.rows()) = V2;
    V.bottomRows(V2.rows()) *= R.transpose();
    V.bottomRows(V2.rows()).array().rowwise() += (pings[i].pos_ - offset).transpose().array();
    viewer.data().set_vertices(V);
}

void ViewDraper::visualize_intensities()
{
    set_texture(texture_image, bounds);
}
