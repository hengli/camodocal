#ifndef COSTFUNCTIONFACTORY_H
#define COSTFUNCTIONFACTORY_H

#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>

#include "camodocal/camera_models/Camera.h"

namespace ceres
{
    class CostFunction;
}

namespace camodocal
{

enum
{
    CAMERA_INTRINSICS = 0x1,
    CAMERA_EXTRINSICS = 0x2,
    POINT_3D = 0x4,
    ODOMETRY_INTRINSICS = 0x8,
    ODOMETRY_3D_EXTRINSICS = 0x10,
    ODOMETRY_6D_EXTRINSICS = 0x20,
    CAMERA_ODOMETRY_EXTRINSICS = 0x40
};

enum
{
    OPTIMIZE_CAMERA_ODOMETRY_Z = 0x1,
    OPTIMIZE_ODOMETRY_3D = 0x2,
    OPTIMIZE_ODOMETRY_6D = 0x4
};

class CostFunctionFactory
{
public:
    CostFunctionFactory();

    static boost::shared_ptr<CostFunctionFactory> instance(void);

    ceres::CostFunction* generateCostFunction(const CameraConstPtr& camera,
                                              const Eigen::Vector3d& observed_P,
                                              const Eigen::Vector2d& observed_p,
                                              int flags) const;

    ceres::CostFunction* generateCostFunction(const CameraConstPtr& camera,
                                              const Eigen::Vector2d& observed_p,
                                              int flags, bool optimize_cam_odo_z = true) const;

    ceres::CostFunction* generateCostFunction(const CameraConstPtr& camera,
                                              const Eigen::Vector3d& odo_pos,
                                              const Eigen::Vector3d& odo_att,
                                              const Eigen::Vector2d& observed_p,
                                              int flags, bool optimize_cam_odo_z = true) const;

    ceres::CostFunction* generateCostFunction(const CameraConstPtr& cameraLeft,
                                              const CameraConstPtr& cameraRight,
                                              const Eigen::Vector3d& observed_P,
                                              const Eigen::Vector2d& observed_p_left,
                                              const Eigen::Vector2d& observed_p_right) const;

private:
    static boost::shared_ptr<CostFunctionFactory> m_instance;
};

}

#endif
