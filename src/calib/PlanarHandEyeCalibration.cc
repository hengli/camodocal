#include "camodocal/calib/PlanarHandEyeCalibration.h"

#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <complex>

#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>

#include "../gpl/EigenUtils.h"
#include "ceres/ceres.h"

namespace camodocal
{

class PoseError
{
public:
    PoseError(Eigen::Vector3d r1, Eigen::Vector3d t1,
              Eigen::Vector3d r2, Eigen::Vector3d t2)
        : m_rvec1(r1), m_tvec1(t1), m_rvec2(r2), m_tvec2(t2)
    {}

    template<typename T>
    bool operator() (const T* const q4x1, const T* const t2x1,
                     T* residuals) const
    {
        Eigen::Quaternion<T> q(q4x1[0], q4x1[1], q4x1[2], q4x1[3]);
        Eigen::Matrix<T,3,1> t;
        t << t2x1[0], t2x1[1], T(0);

        Eigen::Matrix<T,3,1> r1 = m_rvec1.cast<T>();
        Eigen::Matrix<T,3,1> t1 = m_tvec1.cast<T>();
        Eigen::Matrix<T,3,1> r2 = m_rvec2.cast<T>();
        Eigen::Matrix<T,3,1> t2 = m_tvec2.cast<T>();

        Eigen::Quaternion<T> q1 = AngleAxisToQuaternion<T>(r1);
        Eigen::Quaternion<T> q2 = AngleAxisToQuaternion<T>(r2);

        Eigen::Matrix<T,3,3> R2 = AngleAxisToRotationMatrix<T>(r2);

        T q_coeffs[4] = {q.w(), q.x(), q.y(), q.z()};
        Eigen::Matrix<T,3,3> R = QuaternionToRotation<T>(q_coeffs);

        residuals[0] = ((R2 - Eigen::Matrix<T,3,3>::Identity()) * t
                        - (R * t1) + t2).norm();
        residuals[1] = ((q2 * q).coeffs() - (q * q1).coeffs()).norm();

        return true;
    }

private:
    Eigen::Vector3d m_rvec1, m_rvec2, m_tvec1, m_tvec2;
};

PlanarHandEyeCalibration::PlanarHandEyeCalibration()
 : m_verbose(false)
{

}

bool
PlanarHandEyeCalibration::addMotions(const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> >& H1,
                                     const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> >& H2)
{
    if (H1.size() != H2.size())
    {
        return false;
    }

    for (size_t i = 0; i < H1.size(); ++i)
    {
        Eigen::Matrix3d R1 = H1.at(i).block<3,3>(0,0);
        Eigen::Vector3d t1 = H1.at(i).block<3,1>(0,3);

        Motion motion1;
        motion1.rotation = RotationToAngleAxis(R1);
        motion1.translation = t1;

        m_motions1.push_back(motion1);

        Eigen::Matrix3d R2 = H2.at(i).block<3,3>(0,0);
        Eigen::Vector3d t2 = H2.at(i).block<3,1>(0,3);

        Motion motion2;
        motion2.rotation = RotationToAngleAxis(R2);
        motion2.translation = t2;

        m_motions2.push_back(motion2);
    }

    return true;
}

size_t
PlanarHandEyeCalibration::getMotionCount(void) const
{
    return m_motions1.size();
}

bool
PlanarHandEyeCalibration::calibrate(Eigen::Matrix4d& H_12)
{
    if (m_motions1.size() != m_motions2.size())
    {
        return false;
    }

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > rvecs1, tvecs1, rvecs2, tvecs2;

    for (size_t i = 0; i < m_motions1.size(); ++i)
    {
        rvecs1.push_back(m_motions1.at(i).rotation);
        tvecs1.push_back(m_motions1.at(i).translation);
        rvecs2.push_back(m_motions2.at(i).rotation);
        tvecs2.push_back(m_motions2.at(i).translation);
    }

    return estimate(rvecs1, tvecs1, rvecs2, tvecs2, H_12);
}

bool
PlanarHandEyeCalibration::getVerbose(void)
{
    return m_verbose;
}

void 
PlanarHandEyeCalibration::setVerbose(bool on)
{
    m_verbose = on;
}

bool 
PlanarHandEyeCalibration::estimate(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& rvecs1,
                                   const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& tvecs1,
                                   const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& rvecs2,
                                   const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& tvecs2,
                                   Eigen::Matrix4d& H_12) const
{
    // Estimate R_yx first
    Eigen::Matrix3d R_yx;
    estimateRyx(rvecs1, tvecs1, rvecs2, tvecs2, R_yx);

    int motionCount = rvecs1.size();

    Eigen::MatrixXd G = Eigen::MatrixXd::Zero(motionCount * 2, 4);
    Eigen::MatrixXd w(motionCount * 2, 1);

    for (int i = 0; i < motionCount; ++i)
    {
        const Eigen::Vector3d& rvec1 = rvecs1.at(i);
        const Eigen::Vector3d& tvec1 = tvecs1.at(i);
        const Eigen::Vector3d& rvec2 = rvecs2.at(i);
        const Eigen::Vector3d& tvec2 = tvecs2.at(i);

        // Remove zero rotation.
        if (rvec1.norm() == 0 || rvec2.norm() == 0)
        {
            continue;
        }

        Eigen::Quaterniond q2;
        q2 = Eigen::AngleAxisd(rvec2.norm(), rvec2.normalized());

        Eigen::Matrix2d J;
        J = q2.toRotationMatrix().block<2,2>(0,0) - Eigen::Matrix2d::Identity();

        // project tvec1 to plane with normal defined by 3rd row of R_yx
        Eigen::Vector3d n;
        n = R_yx.row(2);

        Eigen::Vector3d pi = R_yx * (tvec1 - tvec1.dot(n) * n);

        Eigen::Matrix2d K;
        K << pi(0), -pi(1), pi(1), pi(0);

        G.block<2,2>(i * 2, 0) = J;
        G.block<2,2>(i * 2, 2) = K;

        w.block<2,1>(i * 2, 0) = tvec2.block<2,1>(0,0);
    }

    Eigen::Vector4d m;
    m = G.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(w);

    Eigen::Vector2d t(-m(0), -m(1));

    double alpha = atan2(m(3), m(2));

    H_12.setIdentity();
    H_12.block<3,3>(0,0) = Eigen::AngleAxisd(alpha, Eigen::Vector3d::UnitZ()) * R_yx;
    H_12.block<2,1>(0,3) = t;

    if (m_verbose)
    {
        std::cout << "# INFO: Before refinement:" << std::endl;
        std::cout << "H_12 = " << std::endl;
        std::cout << H_12 << std::endl;
    }

    refineEstimate(H_12, rvecs1, tvecs1, rvecs2, tvecs2);

    if (m_verbose)
    {
        std::cout << "# INFO: After refinement:" << std::endl;
        std::cout << "H_12 = " << std::endl;
        std::cout << H_12 << std::endl;
    }

    return true;
}

bool
PlanarHandEyeCalibration::estimateRyx(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& rvecs1,
                                      const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& tvecs1,
                                      const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& rvecs2,
                                      const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& tvecs2,
                                      Eigen::Matrix3d& R_yx) const
{
    size_t motionCount = rvecs1.size();

    Eigen::MatrixXd M(motionCount * 4, 4);
    M.setZero();

    for (size_t i = 0; i < rvecs1.size(); ++i)
    {
        const Eigen::Vector3d& rvec1 = rvecs1.at(i);
        const Eigen::Vector3d& tvec1 = tvecs1.at(i);
        const Eigen::Vector3d& rvec2 = rvecs2.at(i);
        const Eigen::Vector3d& tvec2 = tvecs2.at(i);

        // Remove zero rotation.
        if (rvec1.norm() == 0 || rvec2.norm() == 0)
        {
            continue;
        }

        Eigen::Quaterniond q1;
        q1 = Eigen::AngleAxisd(rvec1.norm(), rvec1.normalized());

        Eigen::Quaterniond q2;
        q2 = Eigen::AngleAxisd(rvec2.norm(), rvec2.normalized());

        M.block<4,4>(i * 4, 0) = QuaternionMultMatLeft(q2) - QuaternionMultMatRight(q1);
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::Vector4d t1 = svd.matrixV().block<4,1>(0,2);
    Eigen::Vector4d t2 = svd.matrixV().block<4,1>(0,3);

    // solve constraint for q_yz: xy = -zw
    double s[2];
    if (!solveQuadraticEquation(t1(0) * t1(1) + t1(2) * t1(3),
                                t1(0) * t2(1) + t1(1) * t2(0) + t1(2) * t2(3) + t1(3) * t2(2),
                                t2(0) * t2(1) + t2(2) * t2(3),
                                s[0], s[1]))
    {
        std::cout << "# ERROR: Quadratic equation cannot be solved due to negative determinant." << std::endl;
        return false;
    }

    Eigen::Matrix3d R_yxs[2];
    double yaw[2];
    
    for (int i = 0; i < 2; ++i)
    {
        double t = s[i] * s[i] * t1.dot(t1) + 2 * s[i] * t1.dot(t2) + t2.dot(t2);

        // solve constraint ||q_yx|| = 1
        double b = sqrt(1.0 / t);
        double a = s[i] * b;

        Eigen::Quaterniond q_yx;
        q_yx.coeffs() = a * t1 + b * t2;
        R_yxs[i] = q_yx.toRotationMatrix();

        double r, p;
        mat2RPY(R_yxs[i], r, p, yaw[i]);
    }
    if (fabs(yaw[0]) < fabs(yaw[1]))
    {
        R_yx = R_yxs[0];
    }
    else
    {
        R_yx = R_yxs[1];
    }
    return true;
}

void 
PlanarHandEyeCalibration::refineEstimate(Eigen::Matrix4d& H_12,
                                         const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& rvecs1,
                                         const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& tvecs1,
                                         const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& rvecs2,
                                         const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& tvecs2) const
{
    Eigen::Quaterniond q(H_12.block<3,3>(0,0));
    double p[6] = {q.w(), q.x(), q.y(), q.z(), H_12(0,3), H_12(1,3)};

    ceres::Problem problem;

    for (size_t i = 0; i < rvecs1.size(); ++i)
    {
        ceres::CostFunction* costFunction =
            // t is only flexible on x and y.
            new ceres::AutoDiffCostFunction<PoseError, 2, 4, 2>(
                new PoseError(rvecs1.at(i), tvecs1.at(i), rvecs2.at(i), tvecs2.at(i)));

        problem.AddResidualBlock(costFunction, NULL, p, p + 4);
    }

    ceres::LocalParameterization* quaternionParameterization =
        new ceres::QuaternionParameterization;

    problem.SetParameterization(p, quaternionParameterization);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.jacobi_scaling = true;
    options.max_num_iterations = 2500;
//    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    if (m_verbose)
    {
        std::cout << summary.BriefReport() << std::endl;
    }

    q = Eigen::Quaterniond(p[0], p[1], p[2], p[3]);
    H_12.block<2,1>(0,3) << p[4], p[5];
    H_12.block<3,3>(0,0) = q.toRotationMatrix();
}

bool
PlanarHandEyeCalibration::solveQuadraticEquation(double a, double b, double c, double& x1, double& x2) const
{
    if (fabs(a) < 1e-12)
    {
        x1 = x2 = -c / b;
        return true;
    }
    double delta2 = b * b - 4.0 * a * c;

    if (delta2 < 0.0)
    {
        return false;
    }

    double delta = sqrt(delta2);

    x1 = (-b + delta) / (2.0 * a);
    x2 = (-b - delta) / (2.0 * a);

    return true;
}

}
