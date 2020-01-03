#ifndef BATHY_TRACER_H
#define BATHY_TRACER_H

#include <Eigen/Dense>
#include <igl/embree/EmbreeIntersector.h>

class BathyTracer
{
private:

    igl::embree::EmbreeIntersector embree;
    Eigen::Vector3d first_V;
    Eigen::Vector3i first_F;

public:

    BathyTracer()
    {
        first_V.setZero();
        first_F.setZero();
    }

    std::tuple<Eigen::MatrixXd, Eigen::MatrixXi> compute_hits(const Eigen::Vector3d& sensor_origin, const Eigen::MatrixXd& dirs, const Eigen::MatrixXd& V, const Eigen::MatrixXi& F);

    Eigen::MatrixXd ray_mesh_intersection(
        const Eigen::MatrixXd& V_source,
        const Eigen::MatrixXd& N_source,
        const Eigen::MatrixXd& V_target,
        const Eigen::MatrixXi& F_target);

    double depth_mesh_underneath_vehicle(const Eigen::Vector3d& origin,
                                         const Eigen::MatrixXd& V_target,
                                         const Eigen::MatrixXi& F_target);
};

#endif // BATHY_TRACER_H
