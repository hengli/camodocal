#ifndef LOCATIONRECOGNITION_H
#define LOCATIONRECOGNITION_H

#include "camodocal/sparse_graph/SparseGraph.h"
#include "../dbow2/DBoW2/DBoW2.h"
#include "../dbow2/DUtils/DUtils.h"
#include "../dbow2/DUtilsCV/DUtilsCV.h"
#include "../dbow2/DVision/DVision.h"

namespace camodocal
{

class LocationRecognition
{
public:
    LocationRecognition();

    void setup(const SparseGraph& graph);

    void knnMatch(const FrameConstPtr& frame, int k, std::vector<FrameTag>& matches) const;
    void knnMatch(const FrameConstPtr& frame, int k, std::vector<FramePtr>& matches) const;

private:
    std::vector<std::vector<float> > frameToBOW(const FrameConstPtr& frame) const;

    Surf64Database m_db;

    std::vector<FrameTag> m_frameTags;
    std::vector<FramePtr> m_frames;
    boost::unordered_map<const Frame*,FrameTag> m_frameMap;
};

}

#endif
