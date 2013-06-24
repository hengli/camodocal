#ifndef UTILS_H
#define UTILS_H

#include "camodocal/calib/SensorDataBuffer.h"
#include "camodocal/sparse_graph/Odometer.h"
#include "camodocal/sparse_graph/SparseGraph.h"

namespace camodocal
{

bool
interpolateOdometer(SensorDataBuffer<OdometerPtr>& odometerBuffer,
                    uint64_t timestamp, OdometerPtr& interpOdo);

bool
interpolatePose(SensorDataBuffer<PosePtr>& poseBuffer,
                uint64_t timestamp, PosePtr& interpPose);

bool
interpolatePoseOffline(SensorDataBuffer<PosePtr>& poseBuffer,
                uint64_t timestamp, PosePtr& interpPose);

}

#endif
