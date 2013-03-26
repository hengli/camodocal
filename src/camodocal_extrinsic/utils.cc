#include "utils.h"

#include "../gpl/gpl.h"

namespace camodocal
{

bool
interpolateOdometer(SensorDataBuffer<OdometerPtr>& odometerBuffer,
                    uint64_t timestamp, OdometerPtr& interpOdo)
{
    OdometerPtr prev, next;

    if (!odometerBuffer.nearest(timestamp, prev, next))
    {
        return false;
    }

    interpOdo = OdometerPtr(new Odometer);
    interpOdo->timeStamp() = timestamp;

    if (prev->position() == next->position())
    {
        interpOdo->position() = prev->position();
        interpOdo->yaw() = prev->yaw();

        return true;
    }

    double t = static_cast<double>(timestamp - prev->timeStamp()) / (next->timeStamp() - prev->timeStamp());

    interpOdo->position() = Eigen::Vector2d(t * (next->x() - prev->x()) + prev->x(),
                                            t * (next->y() - prev->y()) + prev->y());

    prev->yaw() = normalizeTheta(prev->yaw());
    next->yaw() = normalizeTheta(next->yaw());

    interpOdo->yaw() = t * normalizeTheta(next->yaw() - prev->yaw()) + prev->yaw();

    return true;
}

bool
interpolatePose(SensorDataBuffer<PosePtr>& poseBuffer,
                uint64_t timestamp, PosePtr& interpPose)
{
    PosePtr prev, next;

    if (!poseBuffer.nearest(timestamp, prev, next))
    {
        return false;
    }

    interpPose = PosePtr(new Pose);
    interpPose->timeStamp() = timestamp;

    if (prev->translation() == next->translation())
    {
        interpPose->rotation() = prev->rotation();
        interpPose->translation() = prev->translation();

        return true;
    }

    double t = static_cast<double>(timestamp - prev->timeStamp()) / (next->timeStamp() - prev->timeStamp());

    interpPose->rotation() = next->rotation().slerp(t, prev->rotation());
    interpPose->rotation().normalize();

    for (int i = 0; i < 3; ++i)
    {
        interpPose->translation()(i) = t * (next->translation()(i) - prev->translation()(i)) + prev->translation()(i);
    }

    return true;
}

}

