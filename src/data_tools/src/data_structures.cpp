#include <data_tools/data_structures.h>
#include <data_tools/colormap.h>

#include <opencv2/imgproc/imgproc.hpp>

using namespace std;

// instantiate all versions needed to read the structs
template nav_entry::EntriesT read_data<nav_entry::EntriesT>(const boost::filesystem::path& path);
template mbes_ping::PingsT read_data<mbes_ping::PingsT>(const boost::filesystem::path& path);
template pt_submaps read_data<pt_submaps>(const boost::filesystem::path& path);
template gp_submaps read_data<gp_submaps>(const boost::filesystem::path& path);
template track_error_benchmark read_data<track_error_benchmark>(const boost::filesystem::path& path);

// instantiate all versions needed to write the structs
template void write_data<nav_entry::EntriesT>(nav_entry::EntriesT& data, const boost::filesystem::path& path);
template void write_data<mbes_ping::PingsT>(mbes_ping::PingsT& data, const boost::filesystem::path& path);
template void write_data<pt_submaps>(pt_submaps& data, const boost::filesystem::path& path);
template void write_data<gp_submaps>(gp_submaps& data, const boost::filesystem::path& path);
template void write_data<track_error_benchmark>(track_error_benchmark& data, const boost::filesystem::path& path);

// res, xmin, ymin, imxmin, imymin
void track_error_benchmark::track_img_params(mbes_ping::PingsT& pings, int rows, int cols)
{
    auto xcomp = [](const mbes_ping& p1, const mbes_ping& p2) {
        return p1.pos_[0] < p2.pos_[0];
    };
    auto ycomp = [](const mbes_ping& p1, const mbes_ping& p2) {
        return p1.pos_[1] < p2.pos_[1];
    };
    double maxx = std::max_element(pings.begin(), pings.end(), xcomp)->pos_[0];
    double minx = std::min_element(pings.begin(), pings.end(), xcomp)->pos_[0];
    double maxy = std::max_element(pings.begin(), pings.end(), ycomp)->pos_[1];
    double miny = std::min_element(pings.begin(), pings.end(), ycomp)->pos_[1];

    cout << minx << ", " << maxx << ", " << miny << ", " << maxy << endl;

    double xres = double(cols)/(maxx - minx);
    double yres = double(rows)/(maxy - miny);

    double res = std::min(xres, yres);

    double x0 = .5*(double(cols) - res*(maxx-minx));
    double y0 = .5*(double(rows) - res*(maxy-miny));

    cout << xres << ", " << yres << endl;

    params = array<double, 5>{res, minx, miny, x0, y0};
    track_img = cv::Mat(rows, cols, CV_8UC3, cv::Scalar(255, 255, 255));
}

void track_error_benchmark::draw_track_img(mbes_ping::PingsT& pings)
{
    nbr_tracks_drawn += 1; // we should based the color on this instead

    double res, minx, miny, x0, y0;
    res = params[0]; minx = params[1]; miny = params[2]; x0 = params[3]; y0 = params[4];

    vector<cv::Point2f> curve_points;
    for (const mbes_ping& ping : pings) {
        cv::Point2f pt(x0+res*(ping.pos_[0]-minx), y0+res*(ping.pos_[1]-miny));
        curve_points.push_back(pt);
        //cout << pt << endl;
    }

    cv::Mat curve(curve_points, true);
    curve.convertTo(curve, CV_32S); //adapt type for polylines
    cv::polylines(track_img, curve, false, cv::Scalar(colormap[nbr_tracks_drawn][2], colormap[nbr_tracks_drawn][1], colormap[nbr_tracks_drawn][0]), 1, CV_AA);
    
    track_img_path = "track.png";
}

void track_error_benchmark::draw_track_img(pt_submaps::TransT& positions)
{
    nbr_tracks_drawn += 1; // we should based the color on this instead

    double res, minx, miny, x0, y0;
    res = params[0]; minx = params[1]; miny = params[2]; x0 = params[3]; y0 = params[4];

    vector<cv::Point2f> curve_points;
    for (const Eigen::Vector3d pos : positions) {
        cv::Point2f pt(x0+res*(pos[0]-minx), y0+res*(pos[1]-miny));
        curve_points.push_back(pt);
        //cout << pt << endl;
    }

    cv::Mat curve(curve_points, true);
    curve.convertTo(curve, CV_32S); //adapt type for polylines
    cv::polylines(track_img, curve, false, cv::Scalar(colormap[nbr_tracks_drawn][2], colormap[nbr_tracks_drawn][1], colormap[nbr_tracks_drawn][0]), 1, CV_AA);
    
    track_img_path = "track.png";
}

double track_error_benchmark::compute_rms_error(pt_submaps::TransT& corrected_track)
{
    if (corrected_track.size() != gt_track.size()) {
        cout << "Track length not same as ground truth!" << endl;
        exit(-1);
    }

    double rms_error = 0.; double count = 0.;
    for (int i = 0; i < gt_track.size(); ++i) {
        rms_error += (corrected_track[i]-gt_track[i]).squaredNorm();
        count += 1.;
    }

    return sqrt(rms_error/count);
}
