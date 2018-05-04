#ifndef SUBMAPS_H
#define SUBMAPS_H

#include <Eigen/Dense>
#include <boost/filesystem.hpp>

using SubmapsT = std::vector<std::vector<Eigen::MatrixXd, Eigen::aligned_allocator<Eigen::MatrixXd> > >;

Eigen::MatrixXd read_submap(const boost::filesystem::path& filename);
SubmapsT read_submaps(const boost::filesystem::path& folder);
void visualize_submaps(SubmapsT& submaps);
void visualize_submap(Eigen::MatrixXd& points);

#endif // SUBMAPS_H
