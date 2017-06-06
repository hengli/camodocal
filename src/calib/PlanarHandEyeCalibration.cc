#include "camodocal/calib/PlanarHandEyeCalibration.h"

#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <complex>

#include <Eigen/Eigen>
#if defined(HAVE_OPENCV2) || defined(HAVE_OPENCV3)
#include <opencv2/core/eigen.hpp>
#endif //  defined(HAVE_OPENCV2) || defined(HAVE_OPENCV3)

#include "camodocal/EigenUtils.h"
#include "ceres/ceres.h"

namespace camodocal
{

class PlanarPoseError
{
public:
    PlanarPoseError(Eigen::Quaterniond q1, Eigen::Vector3d t1,
                    Eigen::Quaterniond q2, Eigen::Vector3d t2)
     : m_q1(q1), m_t1(t1), m_q2(q2), m_t2(t2)
    {}

    template<typename T>
    bool operator() (const T* const q4x1, const T* const t2x1,
                     T* residuals) const
    {
        Eigen::Quaternion<T> q(q4x1[0], q4x1[1], q4x1[2], q4x1[3]);
        Eigen::Matrix<T,3,1> t;
        t << t2x1[0], t2x1[1], T(0);

        Eigen::Quaternion<T> q1 = m_q1.cast<T>();
        Eigen::Matrix<T,3,1> t1 = m_t1.cast<T>();
        Eigen::Quaternion<T> q2 = m_q2.cast<T>();
        Eigen::Matrix<T,3,1> t2 = m_t2.cast<T>();

        T q1_coeffs[4] = {q1.w(), q1.x(), q1.y(), q1.z()};
        Eigen::Matrix<T,3,3> R1 = QuaternionToRotation<T>(q1_coeffs);

        T q_coeffs[4] = {q.w(), q.x(), q.y(), q.z()};
        Eigen::Matrix<T,3,3> R = QuaternionToRotation<T>(q_coeffs);

        Eigen::Matrix<T,3,1> t_err = (R1 - Eigen::Matrix<T,3,3>::Identity()) * t
                                     - (R * t2) + t1;

        Eigen::Quaternion<T> q_err = q.conjugate() * q1 * q * q2.conjugate();

        T q_err_coeffs[4] = {q_err.w(), q_err.x(), q_err.y(), q_err.z()};
        Eigen::Matrix<T,3,3> R_err = QuaternionToRotation<T>(q_err_coeffs);

        T roll, pitch, yaw;
        mat2RPY(R_err, roll, pitch, yaw);

        residuals[0] = t_err(0);
        residuals[1] = t_err(1);
        residuals[2] = t_err(2);
        residuals[3] = roll;
        residuals[4] = pitch;
        residuals[5] = yaw;

        return true;
    }

private:
    Eigen::Quaterniond m_q1, m_q2;
    Eigen::Vector3d m_t1, m_t2;
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
        motion1.rotation = Eigen::Quaterniond(R1);
        motion1.translation = t1;

        m_motions1.push_back(motion1);

        Eigen::Matrix3d R2 = H2.at(i).block<3,3>(0,0);
        Eigen::Vector3d t2 = H2.at(i).block<3,1>(0,3);

        Motion motion2;
        motion2.rotation = Eigen::Quaterniond(R2);
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

    std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> > quats1, quats2;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > tvecs1, tvecs2;

    for (size_t i = 0; i < m_motions1.size(); ++i)
    {
        quats1.push_back(m_motions1.at(i).rotation);
        tvecs1.push_back(m_motions1.at(i).translation);
        quats2.push_back(m_motions2.at(i).rotation);
        tvecs2.push_back(m_motions2.at(i).translation);
    }

    return estimate(quats2, tvecs2, quats1, tvecs1, H_12);
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
PlanarHandEyeCalibration::estimate(const std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> >& quats1,
                                   const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& tvecs1,
                                   const std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> >& quats2,
                                   const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& tvecs2,
                                   Eigen::Matrix4d& H_12) const
{
    // Estimate R_yx first
    Eigen::Matrix3d R_yx;
    estimateRyx(quats1, tvecs1, quats2, tvecs2, R_yx);

    int motionCount = quats1.size();

    Eigen::MatrixXd G = Eigen::MatrixXd::Zero(motionCount * 2, 4);
    Eigen::MatrixXd w(motionCount * 2, 1);

    for (int i = 0; i < motionCount; ++i)
    {
        const Eigen::Quaterniond& q1 = quats1.at(i);
        const Eigen::Vector3d& tvec1 = tvecs1.at(i);
        const Eigen::Quaterniond& q2 = quats2.at(i);
        const Eigen::Vector3d& tvec2 = tvecs2.at(i);

        Eigen::Matrix2d J;
        J = q1.toRotationMatrix().block<2,2>(0,0) - Eigen::Matrix2d::Identity();

        // project tvec1 to plane with normal defined by 3rd row of R_yx
        Eigen::Vector3d n;
        n = R_yx.row(2);

        Eigen::Vector3d pi = R_yx * (tvec2 - tvec2.dot(n) * n);

        Eigen::Matrix2d K;
        K << pi(0), -pi(1), pi(1), pi(0);

        G.block<2,2>(i * 2, 0) = J;
        G.block<2,2>(i * 2, 2) = K;

        w.block<2,1>(i * 2, 0) = tvec1.block<2,1>(0,0);
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

    refineEstimate(H_12, quats1, tvecs1, quats2, tvecs2);

    if (m_verbose)
    {
        std::cout << "# INFO: After refinement:" << std::endl;
        std::cout << "H_12 = " << std::endl;
        std::cout << H_12 << std::endl;
    }

    return true;
}

bool
PlanarHandEyeCalibration::estimateRyx(const std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> >& quats1,
                                      const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& tvecs1,
                                      const std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> >& quats2,
                                      const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& tvecs2,
                                      Eigen::Matrix3d& R_yx) const
{
    size_t motionCount = quats1.size();

    Eigen::MatrixXd M(motionCount * 4, 4);
    M.setZero();

    for (size_t i = 0; i < quats1.size(); ++i)
    {
        const Eigen::Quaterniond& q1 = quats1.at(i);
        const Eigen::Vector3d& tvec1 = tvecs1.at(i);
        const Eigen::Quaterniond& q2 = quats2.at(i);
        const Eigen::Vector3d& tvec2 = tvecs2.at(i);

        M.block<4,4>(i * 4, 0) = QuaternionMultMatLeft(q1) - QuaternionMultMatRight(q2);
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
                                         const std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> >& quats1,
                                         const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& tvecs1,
                                         const std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> >& quats2,
                                         const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& tvecs2) const
{
    Eigen::Quaterniond q(H_12.block<3,3>(0,0));
    double q_coeffs[4] = {q.w(), q.x(), q.y(), q.z()};
    double t_coeffs[3] = {H_12(0,3), H_12(1,3), H_12(2,3)};

    ceres::Problem problem;

    for (size_t i = 0; i < quats1.size(); ++i)
    {
        ceres::CostFunction* costFunction =
            // t is only flexible on x and y.
            new ceres::AutoDiffCostFunction<PlanarPoseError, 6, 4, 2>(
                new PlanarPoseError(quats1.at(i), tvecs1.at(i), quats2.at(i), tvecs2.at(i)));

        problem.AddResidualBlock(costFunction, NULL, q_coeffs, t_coeffs);
    }

    ceres::LocalParameterization* quaternionParameterization =
        new ceres::QuaternionParameterization;

    problem.SetParameterization(q_coeffs, quaternionParameterization);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 100;
//    options.minimizer_progress_to_stdout = true;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    if (m_verbose)
    {
        std::cout << summary.BriefReport() << std::endl;
    }

    q = Eigen::Quaterniond(q_coeffs[0], q_coeffs[1], q_coeffs[2], q_coeffs[3]);
    H_12.block<3,1>(0,3) << t_coeffs[0], t_coeffs[1], t_coeffs[2];
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
