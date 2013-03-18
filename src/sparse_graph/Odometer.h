#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <stdint.h>

namespace camodocal
{

class Odometer
{
public:
    Odometer();

    uint64_t& timeStamp(void);
    uint64_t timeStamp(void) const;
    double& x(void);
    double x(void) const;
    double& y(void);
    double y(void) const;
    Eigen::Vector2d& position(void);
    const Eigen::Vector2d& position(void) const;
    double* positionData(void);
    const double* const positionData(void) const;
    double& yaw(void);
    double yaw(void) const;
    double* yawData(void);
    const double* const yawData(void) const;
    Eigen::Matrix4d pose(void) const;

private:
    Eigen::Vector2d mPos;
    double mYaw;
    uint64_t mTimeStamp;

}; 

typedef boost::shared_ptr<Odometer> OdometerPtr;
typedef boost::shared_ptr<const Odometer> OdometerConstPtr;

}

#endif
