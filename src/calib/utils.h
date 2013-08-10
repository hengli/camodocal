#ifndef UTILS_H
#define UTILS_H

#include "camodocal/calib/SensorDataBuffer.h"
#include "camodocal/sparse_graph/Odometry.h"
#include "camodocal/sparse_graph/SparseGraph.h"

namespace camodocal
{

bool
interpolateOdometry(SensorDataBuffer<OdometryPtr>& odometryBuffer,
                    uint64_t timestamp, OdometryPtr& interpOdo);

bool
interpolatePose(SensorDataBuffer<PosePtr>& poseBuffer,
                uint64_t timestamp, PosePtr& interpPose);

}

#endif
