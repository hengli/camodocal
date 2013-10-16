#ifndef POSEGRAPH_H
#define POSEGRAPH_H

#include <camodocal/camera_models/Camera.h>
#include <camodocal/camera_systems/CameraRigExtrinsics.h>
#include <camodocal/pose_graph/DirectedEdge.h>
#include <camodocal/sparse_graph/SparseGraph.h>
#include <vector>

namespace camodocal
{

// forward declaration
class LocationRecognition;

class PoseGraph
{
public:
    PoseGraph(std::vector<CameraPtr>& cameras,
              CameraRigExtrinsics& extrinsics,
              SparseGraph& graph,
              float maxDistanceRatio,
              int minLoopCorrespondences2D3D,
              int nImageMatches,
              double nominalFocalLength);

    void setVerbose(bool onoff);

    void buildEdges(void);

    void optimize(bool useRobustOptimization);

private:
    enum EdgeSwitchState
    {
        DISABLED,
        OFF,
        ON,
    };

    typedef DirectedEdge<Transform, Odometry> Edge;

    std::vector<Edge, Eigen::aligned_allocator<Edge> > findOdometryEdges(void) const;
    std::vector<Edge, Eigen::aligned_allocator<Edge> > findLoopClosureEdges(double reprojErrorThresh = 2.0) const;

    void findLoopClosureEdgesHelper(FrameTag frameTagQuery,
                                    boost::shared_ptr<LocationRecognition> locRec,
                                    std::vector<PoseGraph::Edge, Eigen::aligned_allocator<PoseGraph::Edge> >* edges,
                                    double reprojErrorThresh) const;

    bool iterateEM(void);
    void classifySwitches(void);

    std::vector<CameraPtr> m_cameras;
    CameraRigExtrinsics m_extrinsics;
    SparseGraph& m_graph;
    std::vector<Edge, Eigen::aligned_allocator<Edge> > m_odometryEdges;
    std::vector<Edge, Eigen::aligned_allocator<Edge> > m_loopClosureEdges;
    std::vector<EdgeSwitchState> m_loopClosureEdgeSwitches;

    const double k_lossWidth;
    const int k_minLoopCorrespondences2D3D;
    const float k_maxDistanceRatio;
    const int k_nImageMatches;
    const double k_nominalFocalLength;

    bool m_verbose;
};

}

#endif
