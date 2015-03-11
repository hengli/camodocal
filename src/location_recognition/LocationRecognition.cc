#include "LocationRecognition.h"

namespace camodocal
{

LocationRecognition::LocationRecognition()
{

}

void
LocationRecognition::setup(const SparseGraph& graph)
{
    m_frameTags.clear();
    m_frames.clear();
    m_frameMap.clear();

    Surf64Vocabulary voc;
    voc.load("surf64.yml.gz");

    m_db.setVocabulary(voc);

    // build vocabulary tree
    std::vector<std::vector<std::vector<float> > > features;

    for (size_t segmentId = 0; segmentId < graph.frameSetSegments().size(); ++segmentId)
    {
        const FrameSetSegment& segment = graph.frameSetSegment(segmentId);

        for (size_t frameSetId = 0; frameSetId < segment.size(); ++frameSetId)
        {
            const FrameSetPtr& frameSet = segment.at(frameSetId);

            for (size_t frameId = 0; frameId < frameSet->frames().size(); ++frameId)
            {
                const FramePtr& frame = frameSet->frames().at(frameId);

                if (frame.get() == 0)
                {
                    continue;
                }

                FrameTag tag;
                tag.frameSetSegmentId = segmentId;
                tag.frameSetId = frameSetId;
                tag.frameId = frameId;

                m_frameTags.push_back(tag);

                m_frames.push_back(frame);

                m_frameMap.insert(std::make_pair(frame.get(), tag));

                features.push_back(frameToBOW(frame));
            }
        }
    }

    for (size_t i = 0; i < features.size(); ++i)
    {
        m_db.add(features.at(i));
    }
}

void
LocationRecognition::knnMatch(const FrameConstPtr& frame, int k,
                              std::vector<FrameTag>& matches) const
{
    boost::unordered_map<const Frame*,FrameTag>::const_iterator it = m_frameMap.find(frame.get());
    if (it == m_frameMap.end())
    {
        return;
    }

    FrameTag tagQuery = it->second;

    DBoW2::QueryResults ret;
    m_db.query(frameToBOW(frame), ret, 0);

    matches.clear();
    for (size_t i = 0; i < ret.size(); ++i)
    {
        FrameTag tag = m_frameTags.at(ret.at(i).Id);

        if (tagQuery.frameSetSegmentId == tag.frameSetSegmentId &&
            std::abs(tagQuery.frameSetId - tag.frameSetId) < 30)
        {
            continue;
        }

        matches.push_back(tag);

        if ((int)matches.size() == k)
        {
            return;
        }
    }
}

void
LocationRecognition::knnMatch(const FrameConstPtr& frame, int k,
                              std::vector<FramePtr>& matches) const
{
    boost::unordered_map<const Frame*,FrameTag>::const_iterator it = m_frameMap.find(frame.get());
    if (it == m_frameMap.end())
    {
        return;
    }

    FrameTag tagQuery = it->second;

    DBoW2::QueryResults ret;
    m_db.query(frameToBOW(frame), ret, 0);

    matches.clear();
    for (size_t i = 0; i < ret.size(); ++i)
    {
        FrameTag tag = m_frameTags.at(ret.at(i).Id);

        if (tagQuery.frameSetSegmentId == tag.frameSetSegmentId &&
            std::abs(tagQuery.frameSetId - tag.frameSetId) < 20)
        {
            continue;
        }

        const FramePtr& frame = m_frames.at(ret.at(i).Id);

        matches.push_back(frame);

        if ((int)matches.size() == k)
        {
            return;
        }
    }
}

std::vector<std::vector<float> >
LocationRecognition::frameToBOW(const FrameConstPtr& frame) const
{
    std::vector<std::vector<float> > bow;

    const std::vector<Point2DFeaturePtr>& features2D = frame->features2D();
    for (std::vector<Point2DFeaturePtr>::const_iterator it = features2D.begin();
            it != features2D.end(); ++it)
    {
        const Point2DFeatureConstPtr& feature2D = (*it);
        const cv::Mat& dtor = feature2D->descriptor();

        std::vector<float> w(dtor.cols);
        for (int j = 0; j < dtor.cols; ++j)
        {
            w.at(j) = dtor.at<float>(0,j);
        }

        bow.push_back(w);
    }

    return bow;
}

}
