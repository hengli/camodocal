#include "Odometer.h"

namespace camodocal
{

Odometer::Odometer()
{

}

uint64_t&
Odometer::timeStamp(void)
{
    return mTimeStamp;
}

uint64_t
Odometer::timeStamp(void) const
{
    return mTimeStamp;
}

double& 
Odometer::x(void)
{
    return mPos(0);
}

double
Odometer::x(void) const
{
    return mPos(0);
}

double&
Odometer::y(void)
{
    return mPos(1);
}

double
Odometer::y(void) const
{
    return mPos(1);
}

Eigen::Vector2d&
Odometer::position(void)
{
    return mPos;
}

const Eigen::Vector2d&
Odometer::position(void) const
{
    return mPos;
}

double*
Odometer::positionData(void)
{
    return mPos.data();
}

const double* const
Odometer::positionData(void) const
{
    return mPos.data();
}

double& 
Odometer::yaw(void)
{
    return mYaw; 
}

double
Odometer::yaw(void) const
{
    return mYaw;
}

double*
Odometer::yawData(void)
{
    return &mYaw;
}

const double* const
Odometer::yawData(void) const
{
    return &mYaw;
}

Eigen::Matrix4d
Odometer::pose(void) const
{
    Eigen::Matrix4d odometerPose;
    odometerPose.setIdentity();

    odometerPose.block<2,2>(0,0) << cos(mYaw), -sin(mYaw),
                                    sin(mYaw), cos(mYaw);
    odometerPose.block<2,1>(0,3) << mPos;

    return odometerPose;
}

}
