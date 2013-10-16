#ifndef DIRECTEDEDGE_H
#define DIRECTEDEDGE_H

#include <boost/weak_ptr.hpp>
#include <Eigen/Eigen>

namespace camodocal
{

enum EdgeType
{
    EDGE_ODOMETRY,
    EDGE_LOOP_CLOSURE
};

template<class EdgeT, class VertexT>
class DirectedEdge
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    DirectedEdge();

    DirectedEdge(const boost::weak_ptr<VertexT>& inVertex,
                 const boost::weak_ptr<VertexT>& outVertex);

    boost::weak_ptr<VertexT>& inVertex(void);
    const boost::weak_ptr<const VertexT> inVertex(void) const;

    boost::weak_ptr<VertexT>& outVertex(void);
    const boost::weak_ptr<const VertexT> outVertex(void) const;

    EdgeT& property(void);
    const EdgeT& property(void) const;

    EdgeType& type(void);
    EdgeType type(void) const;

    std::vector<double>& weight(void);
    const std::vector<double>& weight(void) const;

private:
    boost::weak_ptr<VertexT> m_inVertex;
    boost::weak_ptr<VertexT> m_outVertex;

    EdgeT m_property;
    EdgeType m_type;
    std::vector<double> m_weight;
};

template<class EdgeT, class VertexT>
DirectedEdge<EdgeT, VertexT>::DirectedEdge()
 : m_type(EDGE_ODOMETRY)
{

}

template<class EdgeT, class VertexT>
DirectedEdge<EdgeT, VertexT>::DirectedEdge(const boost::weak_ptr<VertexT>& inVertex,
                                           const boost::weak_ptr<VertexT>& outVertex)
 : m_inVertex(inVertex)
 , m_outVertex(outVertex)
 , m_type(EDGE_ODOMETRY)
{

}

template<class EdgeT, class VertexT>
boost::weak_ptr<VertexT>&
DirectedEdge<EdgeT, VertexT>::inVertex(void)
{
    return m_inVertex;
}

template<class EdgeT, class VertexT>
const boost::weak_ptr<const VertexT>
DirectedEdge<EdgeT, VertexT>::inVertex(void) const
{
    return m_inVertex;
}

template<class EdgeT, class VertexT>
boost::weak_ptr<VertexT>&
DirectedEdge<EdgeT, VertexT>::outVertex(void)
{
    return m_outVertex;
}

template<class EdgeT, class VertexT>
const boost::weak_ptr<const VertexT>
DirectedEdge<EdgeT, VertexT>::outVertex(void) const
{
    return m_outVertex;
}

template<class EdgeT, class VertexT>
EdgeT&
DirectedEdge<EdgeT, VertexT>::property(void)
{
    return m_property;
}

template<class EdgeT, class VertexT>
const EdgeT&
DirectedEdge<EdgeT, VertexT>::property(void) const
{
    return m_property;
}

template<class EdgeT, class VertexT>
EdgeType&
DirectedEdge<EdgeT, VertexT>::type(void)
{
    return m_type;
}

template<class EdgeT, class VertexT>
EdgeType
DirectedEdge<EdgeT, VertexT>::type(void) const
{
    return m_type;
}

template<class EdgeT, class VertexT>
std::vector<double>&
DirectedEdge<EdgeT, VertexT>::weight(void)
{
    return m_weight;
}

template<class EdgeT, class VertexT>
const std::vector<double>&
DirectedEdge<EdgeT, VertexT>::weight(void) const
{
    return m_weight;
}

}

#endif
