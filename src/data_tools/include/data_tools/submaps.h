#ifndef SUBMAPS_H
#define SUBMAPS_H

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <boost/filesystem.hpp>

using ObsT = std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd> >;
using SubmapsT = std::vector<std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd> > >;
using MatchesT = std::vector<std::pair<int, int> >; // tells us which maps overlap
using ConstraintsT = std::vector<std::tuple<int, int, Eigen::Vector3d, Eigen::Vector3d> >;
using BBsT = std::vector<Eigen::Matrix2d, Eigen::aligned_allocator<Eigen::Matrix2d> >;
using TransTT = std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >;
using RotsTT = std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d> >;

Eigen::MatrixXd read_submap(const boost::filesystem::path& filename);
SubmapsT read_submaps(const boost::filesystem::path& folder);
MatchesT compute_matches(const TransTT& trans, const RotsTT& rots, const BBsT& bounds);
ConstraintsT compute_binary_constraints(const TransTT& trans, const RotsTT& rots, const ObsT& points);
ConstraintsT compute_binary_constraints(const TransTT& trans, const RotsTT& rots, const ObsT& points, const ObsT& tracks);
void visualize_submaps(SubmapsT& submaps);
void visualize_submap(Eigen::MatrixXd& points);
Eigen::MatrixXd get_points_in_bound_transform(Eigen::MatrixXd points, Eigen::Vector3d& t,
				                              Eigen::Matrix3d& R, Eigen::Vector3d& t_in,
											  Eigen::Matrix3d& R_in, double bound);
Eigen::MatrixXd get_points_in_bound_transform(Eigen::MatrixXd points, Eigen::Vector3d& t,
				                              Eigen::Matrix3d& R, Eigen::Vector3d& t_in,
											  Eigen::Matrix3d& R_in, Eigen::Matrix2d& bounds);

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
