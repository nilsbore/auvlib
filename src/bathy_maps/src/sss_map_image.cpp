#include <bathy_maps/sss_map_image.h>

using namespace std;

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
                double fraction_zeros = (view.array() == 0).mean(); // / patch_area;
                if (std::isinf(view.mean()) || fraction_zeros > 0.2) {
                    continue;
                }

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
