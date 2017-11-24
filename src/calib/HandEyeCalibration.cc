#include "camodocal/calib/HandEyeCalibration.h"

#include <iostream>
#include <boost/throw_exception.hpp>

#include "ceres/ceres.h"
#include "camodocal/calib/DualQuaternion.h"
#include "camodocal/EigenUtils.h"

namespace camodocal
{

/// @todo there may be an alignment issue, see http://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html
class PoseError
{
public:
    PoseError(Eigen::Vector3d r1, Eigen::Vector3d t1,
              Eigen::Vector3d r2, Eigen::Vector3d t2)
        : m_rvec1(r1), m_rvec2(r2), m_tvec1(t1), m_tvec2(t2)
    {}

    template<typename T>
    bool operator() (const T* const q4x1, const T* const t3x1, T* residual) const
    {
        Eigen::Quaternion<T> q(q4x1[0], q4x1[1], q4x1[2], q4x1[3]);
        Eigen::Matrix<T, 3, 1> t;
        t << t3x1[0], t3x1[1], t3x1[2];

        DualQuaternion<T> dq(q, t);

        Eigen::Matrix<T, 3, 1> r1 = m_rvec1.cast<T>();
        Eigen::Matrix<T, 3, 1> t1 = m_tvec1.cast<T>();
        Eigen::Matrix<T, 3, 1> r2 = m_rvec2.cast<T>();
        Eigen::Matrix<T, 3, 1> t2 = m_tvec2.cast<T>();

        DualQuaternion<T> dq1(AngleAxisToQuaternion<T>(r1), t1);
        DualQuaternion<T> dq2(AngleAxisToQuaternion<T>(r2), t2);
        DualQuaternion<T> dq1_ = dq * dq2 * dq.inverse();

        DualQuaternion<T> diff = (dq1.inverse() * dq1_).log();
        residual[0] = diff.real().squaredNorm() + diff.dual().squaredNorm();

        return true;
    }

private:
    Eigen::Vector3d m_rvec1, m_rvec2, m_tvec1, m_tvec2;
public:
    /// @see http://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

bool HandEyeCalibration::mVerbose = true;

HandEyeCalibration::HandEyeCalibration()
{

}

void
HandEyeCalibration::setVerbose(bool on)
{
    mVerbose = on;
}


/// Reorganize data to prepare for running SVD
/// Daniilidis 1999 Section 6, Equations (31) and (33), on page 291
template <typename T>
static Eigen::MatrixXd ScrewToStransposeBlockofT(
                                                    const Eigen::Matrix<T, 3, 1>& a,
                                                    const Eigen::Matrix<T, 3, 1>& a_prime,
                                                    const Eigen::Matrix<T, 3, 1>& b,
                                                    const Eigen::Matrix<T, 3, 1>& b_prime
                                                    )
{
        Eigen::MatrixXd Stranspose(6,8);
        Stranspose.setZero();

        typedef Eigen::Matrix<T, 3, 1> VecT;
        auto skew_a_plus_b = skew(VecT(a + b));
        auto a_minus_b = a - b;
        Stranspose.block<3,1>( 0, 0) = a_minus_b;
        Stranspose.block<3,3>( 0, 1) = skew_a_plus_b;
        Stranspose.block<3,1>( 3, 0) = a_prime - b_prime;
        Stranspose.block<3,3>( 3, 1) = skew(VecT(a_prime + b_prime));
        Stranspose.block<3,1>( 3, 4) = a_minus_b;
        Stranspose.block<3,3>( 3, 5) = skew_a_plus_b;

        return Stranspose;
}

/// Reorganize data to prepare for running SVD
/// Daniilidis 1999 Section 6, Equations (31) and (33), on page 291
// @pre no zero rotations, thus (rvec1.norm() != 0 && rvec2.norm() != 0) == true
template <typename T>
static Eigen::MatrixXd AxisAngleToSTransposeBlockOfT(
                                                    const Eigen::Matrix<T, 3, 1>& rvec1,
                                                    const Eigen::Matrix<T, 3, 1>& tvec1,
                                                    const Eigen::Matrix<T, 3, 1>& rvec2,
                                                    const Eigen::Matrix<T, 3, 1>& tvec2
                                                    )
{
        double theta1, d1;
        Eigen::Vector3d l1, m1;
        AngleAxisAndTranslationToScrew(rvec1, tvec1, theta1, d1, l1, m1);

        double theta2, d2;
        Eigen::Vector3d l2, m2;
        AngleAxisAndTranslationToScrew(rvec2, tvec2, theta2, d2, l2, m2);

        Eigen::Vector3d a = l1;
        Eigen::Vector3d a_prime = m1;
        Eigen::Vector3d b = l2;
        Eigen::Vector3d b_prime = m2;

        return ScrewToStransposeBlockofT(a,a_prime,b,b_prime);
}

// docs in header
void
HandEyeCalibration::estimateHandEyeScrew(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& rvecs1,
                                         const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& tvecs1,
                                         const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& rvecs2,
                                         const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& tvecs2,
                                         Eigen::Matrix4d& H_12,
                                         bool planarMotion)
{
    int motionCount = rvecs1.size();

    Eigen::MatrixXd T(motionCount * 6, 8);
    T.setZero();

    for (size_t i = 0; i < motionCount; ++i)
    {
        const Eigen::Vector3d& rvec1 = rvecs1.at(i);
        const Eigen::Vector3d& tvec1 = tvecs1.at(i);
        const Eigen::Vector3d& rvec2 = rvecs2.at(i);
        const Eigen::Vector3d& tvec2 = tvecs2.at(i);

        // Skip cases with zero rotation
        if (rvec1.norm() == 0 || rvec2.norm() == 0) continue;

        T.block<6,8>(i * 6, 0) = AxisAngleToSTransposeBlockOfT(rvec1,tvec1,rvec2,tvec2);
    }

    auto dq = estimateHandEyeScrewInitial(T,planarMotion);


    H_12 = dq.toMatrix();
    if (mVerbose)
    {
        std::cout << "# INFO: Before refinement: H_12 = " << std::endl;
        std::cout << H_12 << std::endl;
    }

    estimateHandEyeScrewRefine(dq, rvecs1, tvecs1, rvecs2, tvecs2);

    H_12 = dq.toMatrix();
    if (mVerbose)
    {
        std::cout << "# INFO: After refinement: H_12 = " << std::endl;
        std::cout << H_12 << std::endl;
    }
}


// docs in header
DualQuaterniond
HandEyeCalibration::estimateHandEyeScrewInitial(Eigen::MatrixXd& T,
                                         bool planarMotion)
{


    // dq(r1, t1) = dq * dq(r2, t2) * dq.inv
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(T, Eigen::ComputeFullU | Eigen::ComputeFullV);

    // v7 and v8 span the null space of T, v6 may also be one
    // if rank = 5.
    Eigen::Matrix<double, 8, 1> v6 = svd.matrixV().block<8,1>(0, 5);
    Eigen::Matrix<double, 8, 1> v7 = svd.matrixV().block<8,1>(0,6);
    Eigen::Matrix<double, 8, 1> v8 = svd.matrixV().block<8,1>(0,7);

    // if rank = 5
    if (planarMotion) //(rank == 5)
    {
        if (mVerbose)
        {
            std::cout << "# INFO: No unique solution, returned an arbitrary one. " << std::endl;
        }

        v7 += v6;
    }

    Eigen::Vector4d u1 = v7.block<4,1>(0,0);
    Eigen::Vector4d v1 = v7.block<4,1>(4,0);
    Eigen::Vector4d u2 = v8.block<4,1>(0,0);
    Eigen::Vector4d v2 = v8.block<4,1>(4,0);

    double lambda1 = 0;
    double lambda2 = 0.0;

    if (u1.dot(v1) == 0.0)
    {
        std::swap(u1, u2);
        std::swap(v1, v2);
    }
    if (u1.dot(v1) != 0.0)
    {
        double s[2];
        solveQuadraticEquation(u1.dot(v1), u1.dot(v2) + u2.dot(v1), u2.dot(v2), s[0], s[1]);

        // find better solution for s
        double t[2];
        for (int i = 0; i < 2; ++i)
        {
            t[i] = s[i] * s[i] * u1.dot(u1) + 2 * s[i] * u1.dot(u2) + u2.dot(u2);
        }

        int idx;
        if (t[0] > t[1])
        {
            idx = 0;
        }
        else
        {
            idx = 1;
        }

        double discriminant = 4.0 * square(u1.dot(u2)) - 4.0 * (u1.dot(u1) * u2.dot(u2));
        if (discriminant == 0.0 && mVerbose)
        {
//            std::cout << "# INFO: Noise-free case" << std::endl;
        }

        lambda2 = sqrt(1.0 / t[idx]);
        lambda1 = s[idx] * lambda2;
    }
    else
    {
        if (u1.norm() == 0 && u2.norm() > 0)
        {
            lambda1 = 0;
            lambda2 = 1.0 / u2.norm();
        }
        else if (u2.norm() == 0 && u1.norm() > 0)
        {
            lambda1 = 1.0 / u1.norm();
            lambda2 = 0;
        }
        else
        {
           std::ostringstream ss;

            ss << "camodocal::HandEyeCalibration error: normalization could not be handled. Your rotations and translations are probably either not aligned or not passed in properly.";
            ss << "u1:" << std::endl;
            ss << u1 << std::endl;
            ss << "v1:" << std::endl;
            ss << v1 << std::endl;
            ss << "u2:" << std::endl;
            ss << u2 << std::endl;
            ss << "v2:" << std::endl;
            ss << v2 << std::endl;
            ss << "Not handled yet. Your rotations and translations are probably either not aligned or not passed in properly." << std::endl;

            BOOST_THROW_EXCEPTION(std::runtime_error(ss.str()));
        }
    }

    // rotation
    Eigen::Vector4d q_coeffs = lambda1 * u1 + lambda2 * u2;
    Eigen::Vector4d q_prime_coeffs = lambda1 * v1 + lambda2 * v2;

    Eigen::Quaterniond q(q_coeffs(0), q_coeffs(1), q_coeffs(2), q_coeffs(3));
    Eigen::Quaterniond d(q_prime_coeffs(0), q_prime_coeffs(1), q_prime_coeffs(2), q_prime_coeffs(3));

    return DualQuaterniond(q, d);
}

// docs in header
bool
HandEyeCalibration::solveQuadraticEquation(double a, double b, double c, double& x1, double& x2)
{
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

// docs in header
void
HandEyeCalibration::estimateHandEyeScrewRefine(DualQuaterniond& dq,
                                  const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& rvecs1,
                                  const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& tvecs1,
                                  const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& rvecs2,
                                  const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& tvecs2)
{
    Eigen::Matrix4d H = dq.toMatrix();
    double p[7] = {dq.real().w(), dq.real().x(), dq.real().y(), dq.real().z(),
                   H(0, 3), H(1, 3), H(2, 3)};

    ceres::Problem problem;
    for (size_t i = 0; i < rvecs1.size(); i++)
    {
        // ceres deletes the objects allocated here for the user
        ceres::CostFunction* costFunction =
            new ceres::AutoDiffCostFunction<PoseError, 1, 4, 3>(
                new PoseError(rvecs1[i], tvecs1[i], rvecs2[i], tvecs2[i]));

        problem.AddResidualBlock(costFunction, NULL, p, p + 4);
    }

    // ceres deletes the object allocated here for the user
    ceres::LocalParameterization* quaternionParameterization =
        new ceres::QuaternionParameterization;

    problem.SetParameterization(p, quaternionParameterization);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.jacobi_scaling = true;
    options.max_num_iterations = 500;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    if (mVerbose)
    {
        std::cout << summary.BriefReport() << std::endl;
    }

    Eigen::Quaterniond q(p[0], p[1], p[2], p[3]);
    Eigen::Vector3d t;
    t << p[4], p[5], p[6];
    dq = DualQuaterniond(q, t);
}

}
