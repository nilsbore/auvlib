#ifndef SUBMAPS_H
#define SUBMAPS_H

#include <Eigen/Dense>
#include <iostream>
#include <boost/filesystem.hpp>

using SubmapsT = std::vector<std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd> > >;

Eigen::MatrixXd read_submap(const boost::filesystem::path& filename);
SubmapsT read_submaps(const boost::filesystem::path& folder);
void visualize_submaps(SubmapsT& submaps);
void visualize_submap(Eigen::MatrixXd& points);

template <typename ProcessT>
std::tuple<Eigen::Vector3d, Eigen::Matrix3d> train_gp(Eigen::MatrixXd& points, ProcessT& gp)
{
    std::cout << "Training gaussian process..." << std::endl;
	double meanx = points.col(0).mean();
	double meany = points.col(1).mean();
	double meanz = points.col(2).mean();
	
	points.col(0).array() -= meanx;
	points.col(1).array() -= meany;
	points.col(2).array() -= meanz;

    Eigen::MatrixXd X = points.leftCols(2);
	Eigen::VectorXd y = points.col(2);
	//gp.train_parameters(X, y);
	gp.add_measurements(X, y);

	std::cout << "Done training gaussian process..." << std::endl;

	Eigen::Vector3d t(meanx, meany, meanz);
	Eigen::Matrix3d R;
	R.setIdentity();

    return std::make_tuple(t, R);
}

#endif // SUBMAPS_H
