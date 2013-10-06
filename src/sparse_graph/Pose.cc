#include <camodocal/sparse_graph/Pose.h>

namespace camodocal
{

Pose::Pose()
 : Transform()
 , m_timeStamp(0)
{
    m_covariance.setZero();
}

Pose::Pose(const Eigen::Matrix4d& H)
 : Transform(H)
 , m_timeStamp(0)
{
   m_covariance.setZero();
}

uint64_t&
Pose::timeStamp(void)
{
    return m_timeStamp;
}

uint64_t
Pose::timeStamp(void) const
{
    return m_timeStamp;
}

Eigen::Matrix<double,7,7>&
Pose::covariance(void)
{
    return m_covariance;
}

const Eigen::Matrix<double,7,7>&
Pose::covariance(void) const
{
    return m_covariance;
}

double*
Pose::covarianceData(void)
{
    return m_covariance.data();
}

const double* const
Pose::covarianceData(void) const
{
    return m_covariance.data();
}

}
