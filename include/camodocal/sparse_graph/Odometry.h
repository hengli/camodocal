#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <stdint.h>

namespace camodocal
{

class Odometry
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Odometry();

    uint64_t& timeStamp(void);
    uint64_t timeStamp(void) const;
    double& x(void);
    double x(void) const;
    double& y(void);
    double y(void) const;
    double& z(void);
    double z(void) const;
    Eigen::Vector3d& position(void);
    const Eigen::Vector3d& position(void) const;
    double* positionData(void);
    const double* const positionData(void) const;
    double& yaw(void);
    double yaw(void) const;
    double& pitch(void);
    double pitch(void) const;
    double& roll(void);
    double roll(void) const;
    Eigen::Vector3d& attitude(void);
    const Eigen::Vector3d& attitude(void) const;
    double* attitudeData(void);
    const double* const attitudeData(void) const;
    Eigen::Matrix4d toMatrix(void) const;

    Odometry& operator=(const Odometry& rhs);

private:
    Eigen::Vector3d m_pos;
    Eigen::Vector3d m_att; // 0 - yaw, 1 - pitch, 2 - roll
    uint64_t m_timeStamp;
};

typedef boost::shared_ptr<Odometry> OdometryPtr;
typedef boost::shared_ptr<const Odometry> OdometryConstPtr;

}

#endif
