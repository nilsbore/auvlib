#include <sonar_tracing/snell_ray_tracing.h>
#include <cxxopts.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

int main(int argc, char** argv)
{
    Eigen::MatrixXd end_points(10, 2);
    end_points << 3., -40.,
                  13., -40.,
                  23., -40.,
                  33., -40.,
                  43., -40.,
                  53., -40.,
                  63., -40.,
                  73., -40.,
                  83., -40.,
                  93., -40.;
    Eigen::VectorXd layer_speeds(4); layer_speeds << 0.8, 0.9, 1.1, 1.3;
    Eigen::VectorXd layer_depths(3); layer_depths << -10., -20., -30.;

    // here, we assume that the origin is at (0, 0), and the points all have x > 0
    Eigen::MatrixXd layer_widths = trace_multiple_layers(layer_depths, layer_speeds, end_points);

    const int image_width = 1000;
    const int image_height = 400;

    cv::Mat image(image_height, image_width, CV_8UC3, cv::Scalar(255, 255, 255));

    double vscale = double(image_height)/-45.;
    double hscale = -vscale;

    for (int i = 0; i < end_points.rows(); ++i) {
        cv::Point center(hscale*end_points(i, 0), vscale*end_points(i, 1));
        cv::circle(image, center, 5, cv::Scalar(255, 0, 0), 1);
        cv::line(image, cv::Point(0, 0), center, cv::Scalar(255, 0, 0), 1);
    }

    for (int i = 0; i < layer_depths.rows(); ++i) {
        int row = int(vscale*layer_depths(i));
        cv::line(image, cv::Point(0, row), cv::Point(image_width-1, row), cv::Scalar(0, 0, 255), 1);
    }

    for (int i = 0; i < end_points.rows(); ++i) {
        cv::Point last_point(0, 0);
        for (int j = 0; j < layer_depths.rows(); ++j) {
            cv::Point new_point(hscale*layer_widths(j+1, i), vscale*layer_depths(j));
            cv::line(image, last_point, new_point, cv::Scalar(0, 255, 0), 1);
            last_point = new_point;
        }
        cv::Point new_point(hscale*end_points(i, 0), vscale*end_points(i, 1));
        cv::line(image, last_point, new_point, cv::Scalar(0, 255, 0), 1);
    }

    cv::imshow("Raybending", image);
    cv::waitKey();

    return 0;
}
