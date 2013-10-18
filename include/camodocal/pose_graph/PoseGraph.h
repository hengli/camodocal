#ifndef POSEGRAPH_H
#define POSEGRAPH_H

#include <camodocal/camera_systems/CameraSystem.h>
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
    PoseGraph(CameraSystem& cameraSystem,
              SparseGraph& graph,
              float maxDistanceRatio,
              int minLoopCorrespondences2D3D,
              int nImageMatches,
              double nominalFocalLength);

    void setVerbose(bool onoff);

    void buildEdges(void);

    void optimize(bool useRobustOptimization);

    std::vector<std::pair<Point2DFeaturePtr, Point3DFeaturePtr> > getCorrespondences2D3D(void) const;

private:
    enum EdgeSwitchState
    {
        DISABLED,
        OFF,
        ON,
    };

    typedef DirectedEdge<Transform, Odometry> Edge;

    std::vector<Edge, Eigen::aligned_allocator<Edge> > findOdometryEdges(void) const;
    void findLoopClosures(std::vector<Edge, Eigen::aligned_allocator<Edge> >& loopClosureEdges,
                          std::vector<std::vector<std::pair<Point2DFeaturePtr, Point3DFeaturePtr> > >& correspondences2D3D,
                          double reprojErrorThresh = 2.0) const;

    void findLoopClosuresHelper(FrameTag frameTagQuery,
                                boost::shared_ptr<LocationRecognition> locRec,
                                PoseGraph::Edge* edge,
                                std::vector<std::pair<Point2DFeaturePtr, Point3DFeaturePtr> >* correspondences2D3D,
                                double reprojErrorThresh) const;

    bool iterateEM(bool useRobustOptimization);
    void classifySwitches(void);

#ifdef VCHARGE_VIZ
    void visualizeLoopClosureEdges(void);
#endif

    CameraSystem m_cameraSystem;
    SparseGraph& m_graph;
    std::vector<Edge, Eigen::aligned_allocator<Edge> > m_odometryEdges;
    std::vector<Edge, Eigen::aligned_allocator<Edge> > m_loopClosureEdges;
    std::vector<EdgeSwitchState> m_loopClosureEdgeSwitches;
    std::vector<std::vector<std::pair<Point2DFeaturePtr, Point3DFeaturePtr> > > m_correspondences2D3D;

    const double k_lossWidth;
    const int k_minLoopCorrespondences2D3D;
    const float k_maxDistanceRatio;
    const int k_nImageMatches;
    const double k_nominalFocalLength;

    bool m_verbose;
};

}

#endif
