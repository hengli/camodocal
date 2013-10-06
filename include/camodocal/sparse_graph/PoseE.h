#ifndef POSEE_H
#define POSEE_H

#include <camodocal/sparse_graph/DirectedEdge.h>
#include <camodocal/sparse_graph/Pose.h>
#include <camodocal/sparse_graph/Transform.h>
#include <vector>

namespace camodocal
{

typedef DirectedEdge<Transform, Pose> PoseGraphEdge;

class PoseE: public Pose
{
public:
    PoseE();

    std::vector<PoseGraphEdge, Eigen::aligned_allocator<PoseGraphEdge> >& inEdges(void);
    const std::vector<PoseGraphEdge, Eigen::aligned_allocator<PoseGraphEdge> >& inEdges(void) const;

    std::vector<PoseGraphEdge, Eigen::aligned_allocator<PoseGraphEdge> >& outEdges(void);
    const std::vector<PoseGraphEdge, Eigen::aligned_allocator<PoseGraphEdge> >& outEdges(void) const;

private:
    std::vector<PoseGraphEdge, Eigen::aligned_allocator<PoseGraphEdge> > m_inEdges;
    std::vector<PoseGraphEdge, Eigen::aligned_allocator<PoseGraphEdge> > m_outEdges;
};

typedef boost::shared_ptr<PoseE> PoseEPtr;
typedef boost::shared_ptr<const PoseE> PoseEConstPtr;

}

#endif
