#include "LocationRecognition.h"

namespace camodocal
{

LocationRecognition::LocationRecognition()
{

}

void
LocationRecognition::setup(const SparseGraph& graph,
                           const std::string& databaseDirectory)
{
    m_frameIDs.clear();
    m_frames.clear();

    Surf64Vocabulary voc;
    voc.load("surf64.yml.gz");

    m_db.setVocabulary(voc);

    // build vocabulary tree
    std::vector<std::vector<std::vector<float> > > features;

    for (int cameraIdx = 0; cameraIdx < graph.cameraCount(); ++cameraIdx)
    {
        const std::vector<FrameSegment>& segments = graph.frameSegments(cameraIdx);

        for (size_t segmentIdx = 0; segmentIdx < segments.size(); ++segmentIdx)
        {
            const FrameSegment& segment = segments.at(segmentIdx);

            for (size_t frameIdx = 0; frameIdx < segment.size(); ++frameIdx)
            {
                const FramePtr& frame = segment.at(frameIdx);

                FrameID fid;
                fid.cameraIdx = cameraIdx;
                fid.segmentIdx = segmentIdx;
                fid.frameIdx = frameIdx;
                m_frameIDs.push_back(fid);

                m_frames.push_back(frame);

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
                              std::vector<FrameID>& matches) const
{
    DBoW2::QueryResults ret;
    m_db.query(frameToBOW(frame), ret, k);

    matches.clear();
    for (size_t i = 0; i < ret.size(); ++i)
    {
        FrameID fid = m_frameIDs.at(ret.at(i).Id);

        matches.push_back(fid);
    }
}

void
LocationRecognition::knnMatch(const FrameConstPtr& frame, int k,
                              std::vector<FramePtr>& matches) const
{
    DBoW2::QueryResults ret;
    m_db.query(frameToBOW(frame), ret, k);

    matches.clear();
    for (size_t i = 0; i < ret.size(); ++i)
    {
        const FramePtr& frame = m_frames.at(ret.at(i).Id);

        matches.push_back(frame);
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
