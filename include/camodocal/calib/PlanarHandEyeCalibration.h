#ifndef PLANARHANDEYECALIBRATION_H
#define PLANARHANDEYECALIBRATION_H

#include <Eigen/Eigen>
#include <Eigen/StdVector>

namespace camodocal
{

class PlanarHandEyeCalibration
{
public:
    PlanarHandEyeCalibration();

    bool addMotions(const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> >& H1,
                    const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> >& H2);

    size_t getMotionCount(void) const;

    bool calibrate(Eigen::Matrix4d& H_12);

    bool getVerbose(void);
    void setVerbose(bool on = false);

    bool estimate(const std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> >& quats1,
                  const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& tvecs1,
                  const std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> >& quats2,
                  const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& tvecs2,
                  Eigen::Matrix4d& H_12) const;

private:
    bool estimateRyx(const std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> >& quats1,
                     const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& tvecs1,
                     const std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> >& quats2,
                     const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& tvecs2,
                     Eigen::Matrix3d& R_yx) const;

    void refineEstimate(Eigen::Matrix4d& H_12,
                        const std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> >& quats1,
                        const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& tvecs1,
                        const std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> >& quats2,
                        const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& tvecs2) const;

    // solve ax^2 + bx + c = 0
    bool solveQuadraticEquation(double a, double b, double c, double& x1, double& x2) const;

    typedef struct
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Eigen::Quaterniond rotation;
        Eigen::Vector3d translation;
    } Motion;

    std::vector<Motion, Eigen::aligned_allocator<Motion> > m_motions1;
    std::vector<Motion, Eigen::aligned_allocator<Motion> > m_motions2;

    bool m_verbose;
};

}

#endif
