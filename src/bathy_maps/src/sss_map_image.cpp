#include <bathy_maps/sss_map_image.h>

using namespace std;

sss_map_image_builder::sss_map_image_builder(const sss_map_image::BoundsT& bounds, double resolution, int nbr_pings) : 
    bounds(bounds), resolution(resolution), waterfall_width(2*nbr_pings), waterfall_counter(0)
{
    global_origin = Eigen::Vector3d(bounds(0, 0), bounds(0, 1), 0.);

    image_cols = resolution*(bounds(1, 0) - bounds(0, 0));
    image_rows = resolution*(bounds(1, 1) - bounds(0, 1));

    sss_map_image_counts = Eigen::MatrixXd::Zero(image_rows, image_cols);
    sss_map_image_sums = Eigen::MatrixXd::Zero(image_rows, image_cols);

    sss_waterfall_image = Eigen::MatrixXd::Zero(2000, waterfall_width);
    sss_waterfall_cross_track = Eigen::MatrixXd::Zero(2000, waterfall_width);
    sss_waterfall_depth = Eigen::MatrixXd::Zero(2000, waterfall_width);
}

bool sss_map_image_builder::empty()
{
    return sss_map_image_counts.sum() == 0;
}

Eigen::MatrixXd sss_map_image_builder::downsample_cols(const Eigen::MatrixXd& M, int new_cols)
{
    double factor = double(new_cols)/double(M.cols());
    Eigen::ArrayXXd sums = Eigen::MatrixXd::Zero(M.rows(), new_cols);
    Eigen::ArrayXXd counts = Eigen::MatrixXd::Zero(M.rows(), new_cols);

    int ind;
    for (int i = 0; i < M.rows(); ++i) {
        for (int j = 0; j < M.cols(); ++j) {
            ind = int(factor*j);
            sums(i, ind) += M(i, j);
            counts(i, ind) += 1.;
        }
    }

    counts += (counts == 0).cast<double>();
    sums /= counts;

    return sums.matrix();
}

sss_map_image sss_map_image_builder::finish()
{
    sss_map_image map_image;
    map_image.bounds = bounds;
    if (sss_map_image_counts.sum() > 0) {
        sss_map_image_counts.array() += (sss_map_image_counts.array() == 0).cast<double>();
        map_image.sss_map_image.array() = sss_map_image_sums.array() / sss_map_image_counts.array();
    }
    map_image.pos = poss;
    map_image.sss_waterfall_image = downsample_cols(sss_waterfall_image.topRows(waterfall_counter), 1000).cast<float>();
    //map_image.sss_waterfall_cross_track = sss_waterfall_cross_track.topRows(waterfall_counter);
    map_image.sss_waterfall_depth = downsample_cols(sss_waterfall_depth.topRows(waterfall_counter), 1000).cast<float>();

    return map_image;
}

void sss_map_image_builder::add_waterfall_images(const Eigen::MatrixXd& hits, const Eigen::VectorXi& hits_inds,
                                                 const xtf_sss_ping_side& ping, const Eigen::Vector3d& pos, bool is_left)
{
    if (waterfall_counter > sss_waterfall_image.rows()) {
        sss_waterfall_image.conservativeResize(sss_waterfall_image.rows()+1000, sss_waterfall_image.cols());
        sss_waterfall_cross_track.conservativeResize(sss_waterfall_image.rows()+1000, sss_waterfall_image.cols());
        sss_waterfall_depth.conservativeResize(sss_waterfall_image.rows()+1000, sss_waterfall_image.cols());
    }

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

    for (int i = 0; i < hits.rows(); ++i) {
        int ping_ind = hits_inds[i];
        int col;
        if (is_left) {
            col = waterfall_width/2 + ping_ind;
        }
        else {
            col = waterfall_width/2 - 1 - ping_ind;
        }
        sss_waterfall_cross_track(waterfall_counter, col) = (hits.row(i).head<2>() - pos.head<2>().transpose()).norm();
        sss_waterfall_depth(waterfall_counter, col) = hits(i, 2.) - pos(2);
    }

    if (!is_left) {
        ++waterfall_counter;
    }
}

void sss_map_image_builder::add_hits(const Eigen::MatrixXd& hits, const Eigen::VectorXi& hits_inds,
                                     const xtf_sss_ping_side& ping, const Eigen::Vector3d& pos, bool is_left)
{
    if (hits.rows() == 0) {
        return;
    }

    poss.push_back(pos);

    Eigen::VectorXd intensities = hits.col(3);
    Eigen::MatrixXd points = hits.leftCols<3>();
    // The hits are already compensated to start at bounds.row(0)
    //points.array().rowwise() -= global_origin.array().transpose();
    points.leftCols<2>().array() *= resolution; //*points.leftCols<2>().array();

    std::cout << "Hits rows: " << hits.rows() << std::endl;
    std::cout << "Origin: " << global_origin.transpose() << ", pose: " << pos.transpose() << std::endl;

    int inside_image = 0;
    for (int i = 0; i < points.rows(); ++i) {
        int x = int(points(i, 0));
        int y = int(points(i, 1));
        if (x >= 0 && x < image_cols && y >= 0 && y < image_rows) {
            sss_map_image_sums(y, x) += intensities(i);
            sss_map_image_counts(y, x) += 1.;
            ++inside_image;
        }
    }
    std::cout << "Number inside image: " << inside_image << std::endl;

    add_waterfall_images(hits, hits_inds, ping, pos, is_left);
}

sss_patch_views::ViewsT convert_maps_to_patches(const sss_map_image::ImagesT& map_images, const Eigen::MatrixXd& height_map, double patch_size)
{
    sss_patch_views::ViewsT patches;

    sss_map_image::BoundsT bounds = map_images[0].bounds;
    int image_rows = map_images[0].sss_map_image.rows();
    int image_cols = map_images[0].sss_map_image.cols();

    double resolution = double(image_cols)/(bounds(1, 0) - bounds(0, 0));

    int image_size = patch_size*resolution;

    cout << "Got image size: " << image_size << endl;

    int nbr_patches_x = image_cols / image_size - 1;
    int nbr_patches_y = image_rows / image_size - 1;
    //double patch_area = double(image_size*image_size);

    cout << "Number patches x: " << nbr_patches_x << endl;
    cout << "Number patches y: " << nbr_patches_y << endl;

    cout << "Number rows: " << image_rows << endl;
    cout << "Number cols: " << image_cols << endl;

    for (int i = 0; i < nbr_patches_y; ++i) {
        for (int j = 0; j < nbr_patches_x; ++j) {
            sss_patch_views patch_views;
            patch_views.patch_size = patch_size;
            double x = double(j*image_size + image_size/2)/resolution;
            double y = double(i*image_size + image_size/2)/resolution;
            Eigen::Vector3d origin = Eigen::Vector3d(x, y, 0.);
            patch_views.patch_origin = origin + Eigen::Vector3d(bounds(0, 0), bounds(0, 1), 0.); // find closes to origin in pos
            //patch_views.patch_height = Eigen::MatrixXd::Zero(image_size, image_size);
            patch_views.patch_height = height_map.block(i*image_size, j*image_size, image_size, image_size);
            for (int n = 0; n < map_images.size(); ++n) {
                //cout << "current x start: " << j*image_size << " out of " << map_images[n].sss_map_image.cols() << endl;
                //cout << "current y start: " << i*image_size << " out of " << map_images[n].sss_map_image.rows() << endl;
                Eigen::MatrixXd view = map_images[n].sss_map_image.block(i*image_size, j*image_size, image_size, image_size);
                //cout << "view mean: " << view.mean() << endl;
                double fraction_zeros = (view.array() == 0).cast<double>().mean(); // / patch_area;
                if (fraction_zeros < 1.) {
                    cout << "Mean: " << fraction_zeros << endl;
                }
                if (std::isinf(view.mean()) || fraction_zeros > 0.2) {
                    continue;
                }

                cout << "Accepted mean: " << fraction_zeros << endl;

                patch_views.sss_views.push_back(view);
                auto iter = std::min_element(map_images[n].pos.begin(), map_images[n].pos.end(), [&origin](const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
                    return (p1-origin).squaredNorm() < (p2-origin).squaredNorm();
                });
                int ind = std::distance(map_images[n].pos.begin(), iter);
                //cout << "Origin: " << origin.transpose() << endl;
                //cout << "Got ind: " << ind << " out of " << map_images[n].pos.size() << endl;
                patch_views.patch_view_pos.push_back(*iter-origin);
                ind = std::min(int(map_images[n].pos.size())-3, ind);
                Eigen::Vector3d dir = map_images[n].pos[ind+2]-map_images[n].pos[ind];
                //cout << "p1: " << map_images[n].pos[ind+2].transpose() << endl;
                //cout << "p2: " << map_images[n].pos[ind].transpose() << endl;
                //cout << "Dir: " << dir.transpose() << endl;
                dir.normalize();
                patch_views.patch_view_dirs.push_back(dir);
            }
            if (!patch_views.sss_views.empty()) {
                patches.push_back(patch_views);
            }
        }
    }

    return patches;
}
