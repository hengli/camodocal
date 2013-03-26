#ifndef UTILS_H
#define UTILS_H

#include "camodocal/Odometer.h"
#include "camodocal/SensorDataBuffer.h"
#include "camodocal/SparseGraph.h"

namespace camodocal
{

bool
interpolateOdometer(SensorDataBuffer<OdometerPtr>& odometerBuffer,
                    uint64_t timestamp, OdometerPtr& interpOdo);

bool
interpolatePose(SensorDataBuffer<PosePtr>& poseBuffer,
                uint64_t timestamp, PosePtr& interpPose);

}

#endif
