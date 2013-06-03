#ifndef HANDEYECALIBRATION_H
#define HANDEYECALIBRATION_H

#include <Eigen/Eigen>
#include <opencv2/core/core.hpp>

#include "DualQuaternion.h"

namespace camodocal
{

class HandEyeCalibration
{
public:
    HandEyeCalibration();

    static void estimateHandEyeScrew(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& rvecs1,
                                     const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& tvecs1,
                                     const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& rvecs2,
                                     const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& tvecs2,
                                     Eigen::Matrix4d& H_12, bool planarMotion = false);
    static void setVerbose(bool on = true); 

private:
    // solve ax^2 + bx + c = 0
    static bool solveQuadraticEquation(double a, double b, double c, double& x1, double& x2);

    static void refineHandEye(DualQuaterniond& dq,
                              const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& rvecs1,
                              const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& tvecs1,
                              const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& rvecs2,
                              const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& tvecs2);

    static bool mVerbose;
};

}

#endif
