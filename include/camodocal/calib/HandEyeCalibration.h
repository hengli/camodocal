#ifndef HANDEYECALIBRATION_H
#define HANDEYECALIBRATION_H

#include <Eigen/Eigen>
#include <opencv2/core/core.hpp>

#include "DualQuaternion.h"

namespace camodocal
{

/// @brief Implements Hand Eye Calibration which determines an unknown 3d transform using two stacks of known transforms.
///
/// @see <a href="https://robotics.stackexchange.com/questions/7163/hand-eye-calibration">StackOverflow Explanation of Hand Eye Calibration with CamOdoCal</a>
///
///  Daniilidis, Konstantinos. "Hand-eye calibration using dual quaternions." The International Journal of Robotics Research 18.3 (1999): 286-298.
/// @see <a href="http://citeseerx.ist.psu.edu/viewdoc/summary?doi=10.1.1.136.5873&rank=1">Daniilidis 1999</a>
///

class HandEyeCalibration
{
public:
    HandEyeCalibration();

    
    /// @brief Given two vectors with N equivalent transforms in each index i, estimate the 4x4 Homogeneous transformation matrix between them.
    ///
    /// @param rvecs1 vector of size N containing the unit axis and angle for the first transform
    ///
    /// 1. Each measurement taken at a different time, position, and orientation narrows down the possible transforms that can represent the unknown X
    ///
    /// 2. Record a list of many transforms A and B taken between different time steps, or relative to the first time step
    ///      - Rotations are in AxisAngle = UnitAxis*Angle format, or [x_axis,y_axis,z_axis]*𝜃_angle
    ///         - ||UnitAxis||=1
    ///         - || AxisAngle || = 𝜃_angle
    ///      - Translations are in the normal [x,y,z] format
    /// 3. Pass both vectors into EstimateHandEyeScrew()
    /// 4. Returns X in the form of a 4x4 transform estimate
    static void estimateHandEyeScrew(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& rvecs1,
                                     const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& tvecs1,
                                     const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& rvecs2,
                                     const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& tvecs2,
                                     Eigen::Matrix4d& H_12, bool planarMotion = false);
    
    
    
    
    static void setVerbose(bool on = true); 

private:
    /// @brief solve ax^2 + bx + c = 0
    static bool solveQuadraticEquation(double a, double b, double c, double& x1, double& x2);
    
    /// @brief Initial hand-eye screw estimate using fast but coarse Eigen::JacobiSVD
    static DualQuaterniond estimateHandEyeScrewInitial(Eigen::MatrixXd& T,bool planarMotion);

    /// @brief Refine hand-eye screw estimate using initial coarse estimate and Ceres Solver Library.
    static void estimateHandEyeScrewRefine(DualQuaterniond& dq,
                              const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& rvecs1,
                              const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& tvecs1,
                              const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& rvecs2,
                              const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& tvecs2);

    static bool mVerbose;
};

}

#endif
