#include <camodocal/sparse_graph/PoseE.h>

namespace camodocal
{

PoseE::PoseE()
{

}

std::vector<PoseGraphEdge, Eigen::aligned_allocator<PoseGraphEdge> >&
PoseE::inEdges(void)
{
    return m_inEdges;
}

const std::vector<PoseGraphEdge, Eigen::aligned_allocator<PoseGraphEdge> >&
PoseE::inEdges(void) const
{
    return m_inEdges;
}

std::vector<PoseGraphEdge, Eigen::aligned_allocator<PoseGraphEdge> >&
PoseE::outEdges(void)
{
    return m_outEdges;
}

const std::vector<PoseGraphEdge, Eigen::aligned_allocator<PoseGraphEdge> >&
PoseE::outEdges(void) const
{
    return m_outEdges;
}

}
