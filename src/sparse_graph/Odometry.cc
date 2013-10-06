#include <camodocal/sparse_graph/Odometry.h>

namespace camodocal
{

Odometry::Odometry()
{
    m_pos.setZero();
    m_att.setZero();
    m_timeStamp = 0;
}

uint64_t&
Odometry::timeStamp(void)
{
    return m_timeStamp;
}

uint64_t
Odometry::timeStamp(void) const
{
    return m_timeStamp;
}

double&
Odometry::x(void)
{
    return m_pos(0);
}

double
Odometry::x(void) const
{
    return m_pos(0);
}

double&
Odometry::y(void)
{
    return m_pos(1);
}

double
Odometry::y(void) const
{
    return m_pos(1);
}

double&
Odometry::z(void)
{
    return m_pos(2);
}

double
Odometry::z(void) const
{
    return m_pos(2);
}

Eigen::Vector3d&
Odometry::position(void)
{
    return m_pos;
}

const Eigen::Vector3d&
Odometry::position(void) const
{
    return m_pos;
}

double*
Odometry::positionData(void)
{
    return m_pos.data();
}

const double* const
Odometry::positionData(void) const
{
    return m_pos.data();
}

double&
Odometry::yaw(void)
{
    return m_att(0);
}

double
Odometry::yaw(void) const
{
    return m_att(0);
}

double&
Odometry::pitch(void)
{
    return m_att(1);
}

double
Odometry::pitch(void) const
{
    return m_att(1);
}

double&
Odometry::roll(void)
{
    return m_att(2);
}

double
Odometry::roll(void) const
{
    return m_att(2);
}

Eigen::Vector3d&
Odometry::attitude(void)
{
    return m_att;
}

const Eigen::Vector3d&
Odometry::attitude(void) const
{
    return m_att;
}

double*
Odometry::attitudeData(void)
{
    return m_att.data();
}

const double* const
Odometry::attitudeData(void) const
{
    return m_att.data();
}

Eigen::Matrix4d
Odometry::toMatrix(void) const
{
    Eigen::Matrix4d odometryPose;
    odometryPose.setIdentity();

    Eigen::Matrix3d R_z;
    R_z << cos(m_att(0)), -sin(m_att(0)), 0.0,
           sin(m_att(0)), cos(m_att(0)), 0.0,
           0.0, 0.0, 1.0;

    Eigen::Matrix3d R_y;
    R_y << cos(m_att(1)), 0.0, sin(m_att(1)),
           0.0, 1.0, 0.0,
           -sin(m_att(1)), 0.0, cos(m_att(1));

    Eigen::Matrix3d R_x;
    R_x << 1.0, 0.0, 0.0,
           0.0, cos(m_att(2)), -sin(m_att(2)),
           0.0, sin(m_att(2)), cos(m_att(2));

    odometryPose.block<3,3>(0,0) = R_z * R_y * R_x;
    odometryPose.block<3,1>(0,3) = m_pos;

    return odometryPose;
}

Odometry&
Odometry::operator=(const Odometry& rhs)
{
    if (this != &rhs)
    {
        m_pos = rhs.m_pos;
        m_att = rhs.m_att;
        m_timeStamp = rhs.m_timeStamp;
    }

    return *this;
}

}
