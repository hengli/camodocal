#ifndef POSE_H
#define POSE_H

#include <camodocal/sparse_graph/Transform.h>

namespace camodocal
{

class Pose: public Transform
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Pose();
    Pose(const Eigen::Matrix4d& H);

    uint64_t& timeStamp(void);
    uint64_t timeStamp(void) const;

    Eigen::Matrix<double,7,7>& covariance(void);
    const Eigen::Matrix<double,7,7>& covariance(void) const;
    double* covarianceData(void);
    const double* const covarianceData(void) const;

private:
    uint64_t m_timeStamp;
    Eigen::Matrix<double,7,7> m_covariance;
};

typedef boost::shared_ptr<Pose> PosePtr;
typedef boost::shared_ptr<const Pose> PoseConstPtr;

}

#endif
