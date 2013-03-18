#ifndef UTILS_H
#define UTILS_H

#include "../gpl/SensorDataBuffer.h"
#include "../sparse_graph/Odometer.h"
#include "../sparse_graph/SparseGraph.h"

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
