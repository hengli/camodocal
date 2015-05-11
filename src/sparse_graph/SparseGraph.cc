#include <camodocal/sparse_graph/SparseGraph.h>

#include <boost/filesystem.hpp>
#include <boost/make_shared.hpp>
#include <boost/unordered_set.hpp>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <sstream>

namespace camodocal
{

Frame::Frame()
 : m_cameraId(-1)
{

}

PosePtr&
Frame::cameraPose(void)
{
    return m_cameraPose;
}

PoseConstPtr
Frame::cameraPose(void) const
{
    return m_cameraPose;
}

int&
Frame::cameraId(void)
{
    return m_cameraId;
}

int
Frame::cameraId(void) const
{
    return m_cameraId;
}

OdometryPtr&
Frame::odometryMeasurement(void)
{
    return m_odometryMeasurement;
}

OdometryConstPtr
Frame::odometryMeasurement(void) const
{
    return m_odometryMeasurement;
}

OdometryPtr&
Frame::systemPose(void)
{
    return m_systemPose;
}

OdometryConstPtr
Frame::systemPose(void) const
{
    return m_systemPose;
}

PosePtr&
Frame::gpsInsMeasurement(void)
{
    return m_gpsInsMeasurement;
}

PoseConstPtr
Frame::gpsInsMeasurement(void) const
{
    return m_gpsInsMeasurement;
}

std::vector<Point2DFeaturePtr>&
Frame::features2D(void)
{
    return m_features2D;
}

const std::vector<Point2DFeaturePtr>&
Frame::features2D(void) const
{
    return m_features2D;
}

cv::Mat&
Frame::image(void)
{
    return m_image;
}

const cv::Mat&
Frame::image(void) const
{
    return m_image;
}

Point2DFeature::Point2DFeature()
 : m_index(0)
 , m_bestPrevMatchId(-1)
 , m_bestNextMatchId(-1)
{

}

cv::Mat&
Point2DFeature::descriptor(void)
{
    return m_dtor;
}

const cv::Mat&
Point2DFeature::descriptor(void) const
{
    return m_dtor;
}

cv::KeyPoint&
Point2DFeature::keypoint(void)
{
    return m_keypoint;
}

const cv::KeyPoint&
Point2DFeature::keypoint(void) const
{
    return m_keypoint;
}

unsigned int&
Point2DFeature::index(void)
{
    return m_index;
}

unsigned int
Point2DFeature::index(void) const
{
    return m_index;
}

Point2DFeatureWPtr&
Point2DFeature::prevMatch(void)
{
    return m_prevMatches.at(m_bestPrevMatchId);
}

Point2DFeatureConstWPtr
Point2DFeature::prevMatch(void) const
{
    return m_prevMatches.at(m_bestPrevMatchId);
}

std::vector<Point2DFeatureWPtr>&
Point2DFeature::prevMatches(void)
{
    return m_prevMatches;
}

const std::vector<Point2DFeatureWPtr>&
Point2DFeature::prevMatches(void) const
{
    return m_prevMatches;
}

int&
Point2DFeature::bestPrevMatchId(void)
{
    return m_bestPrevMatchId;
}

int
Point2DFeature::bestPrevMatchId(void) const
{
    return m_bestPrevMatchId;
}

Point2DFeatureWPtr&
Point2DFeature::nextMatch(void)
{
    return m_nextMatches.at(m_bestNextMatchId);
}

Point2DFeatureConstWPtr
Point2DFeature::nextMatch(void) const
{
    return m_nextMatches.at(m_bestNextMatchId);
}

std::vector<Point2DFeatureWPtr>&
Point2DFeature::nextMatches(void)
{
    return m_nextMatches;
}

const std::vector<Point2DFeatureWPtr>&
Point2DFeature::nextMatches(void) const
{
    return m_nextMatches;
}

int&
Point2DFeature::bestNextMatchId(void)
{
    return m_bestNextMatchId;
}

int
Point2DFeature::bestNextMatchId(void) const
{
    return m_bestNextMatchId;
}

Point3DFeaturePtr&
Point2DFeature::feature3D(void)
{
    return m_feature3D;
}

Point3DFeatureConstPtr
Point2DFeature::feature3D(void) const
{
    return m_feature3D;
}

FrameWPtr&
Point2DFeature::frame(void)
{
    return m_frame;
}

FrameConstWPtr
Point2DFeature::frame(void) const
{
    return m_frame;
}

Point3DFeature::Point3DFeature(void)
 : m_attributes(0)
 , m_weight(1.0)
{
    m_point.setZero();
    m_pointCovariance.setZero();
}

Eigen::Vector3d&
Point3DFeature::point(void)
{
    return m_point;
}

const Eigen::Vector3d&
Point3DFeature::point(void) const
{
    return m_point;
}

double*
Point3DFeature::pointData(void)
{
    return m_point.data();
}

const double* const
Point3DFeature::pointData(void) const
{
    return m_point.data();
}

Eigen::Matrix3d&
Point3DFeature::pointCovariance(void)
{
    return m_pointCovariance;
}

const Eigen::Matrix3d&
Point3DFeature::pointCovariance(void) const
{
    return m_pointCovariance;
}

double*
Point3DFeature::pointCovarianceData(void)
{
    return m_pointCovariance.data();
}

const double* const
Point3DFeature::pointCovarianceData(void) const
{
    return m_pointCovariance.data();
}

int&
Point3DFeature::attributes(void)
{
    return m_attributes;
}

int
Point3DFeature::attributes(void) const
{
    return m_attributes;
}

double&
Point3DFeature::weight(void)
{
    return m_weight;
}

double
Point3DFeature::weight(void) const
{
    return m_weight;
}

std::vector<Point2DFeatureWPtr>&
Point3DFeature::features2D(void)
{
    return m_features2D;
}

const std::vector<Point2DFeatureWPtr>&
Point3DFeature::features2D(void) const
{
    return m_features2D;
}

bool
Point3DFeature::removeFeatureObservation(const Point2DFeaturePtr& featureObs)
{
    for (std::vector<Point2DFeatureWPtr>::iterator it = m_features2D.begin();
         it != m_features2D.end(); ++it)
    {
        Point2DFeaturePtr f = it->lock();
        if (!f)
        {
            continue;
        }

        if (f == featureObs)
        {
            m_features2D.erase(it);

            return false;
        }
    }

    return false;
}

FrameSet::FrameSet()
{

}

FramePtr&
FrameSet::frame(int cameraId)
{
    if (cameraId >= m_frames.size())
    {
        m_frames.resize(cameraId + 1);
    }

    return m_frames.at(cameraId);
}

FrameConstPtr
FrameSet::frame(int cameraId) const
{
    return m_frames.at(cameraId);
}

std::vector<FramePtr>&
FrameSet::frames(void)
{
    return m_frames;
}

const std::vector<FramePtr>&
FrameSet::frames(void) const
{
    return m_frames;
}

OdometryPtr&
FrameSet::odometryMeasurement(void)
{
    return m_odometryMeasurement;
}

OdometryConstPtr
FrameSet::odometryMeasurement(void) const
{
    return m_odometryMeasurement;
}

OdometryPtr&
FrameSet::systemPose(void)
{
    return m_systemPose;
}

OdometryConstPtr
FrameSet::systemPose(void) const
{
    return m_systemPose;
}

PosePtr&
FrameSet::gpsInsMeasurement(void)
{
    return m_gpsInsMeasurement;
}

PoseConstPtr
FrameSet::gpsInsMeasurement(void) const
{
    return m_gpsInsMeasurement;
}

SparseGraph::SparseGraph()
{

}

FrameSetSegment&
SparseGraph::frameSetSegment(int segmentId)
{
    if (segmentId >= m_frameSetSegments.size())
    {
        m_frameSetSegments.resize(segmentId + 1);
    }

    return m_frameSetSegments.at(segmentId);
}

const FrameSetSegment&
SparseGraph::frameSetSegment(int segmentId) const
{
    return m_frameSetSegments.at(segmentId);
}

std::vector<FrameSetSegment>&
SparseGraph::frameSetSegments(void)
{
    return m_frameSetSegments;
}

const std::vector<FrameSetSegment>&
SparseGraph::frameSetSegments(void) const
{
    return m_frameSetSegments;
}

size_t
SparseGraph::scenePointCount(void) const
{
    boost::unordered_set<Point3DFeature*> scenePointSet;
    for (size_t i = 0; i < m_frameSetSegments.size(); ++i)
    {
        const FrameSetSegment& segment = m_frameSetSegments.at(i);

        for (size_t j = 0; j < segment.size(); ++j)
        {
            const FrameSetPtr& frameSet = segment.at(j);

            for (size_t k = 0; k < frameSet->frames().size(); ++k)
            {
                const FramePtr& frame = frameSet->frames().at(k);

                if (frame.get() == 0)
                {
                    continue;
                }

                const std::vector<Point2DFeaturePtr>& features2D = frame->features2D();

                for (size_t l = 0; l < features2D.size(); ++l)
                {
                    if (features2D.at(l)->feature3D().get() == 0)
                    {
                        continue;
                    }

                    scenePointSet.insert(features2D.at(l)->feature3D().get());
                }
            }
        }
    }

    return scenePointSet.size();
}

bool
SparseGraph::readFromBinaryFile(const std::string& filename)
{
    boost::filesystem::path filePath(filename);

    boost::filesystem::path rootDir;
    if (filePath.has_parent_path())
    {
        rootDir = filePath.parent_path();
    }
    else
    {
        rootDir = boost::filesystem::path(".");
    }

    m_frameSetSegments.clear();

    // parse binary file
    std::ifstream ifs(filename.c_str(), std::ios::in | std::ios::binary);
    if (!ifs.is_open())
    {
        return false;
    }

    size_t nFrames;
    readData(ifs, nFrames);

    size_t nPoses;
    readData(ifs, nPoses);

    size_t nOdometry;
    readData(ifs, nOdometry);

    size_t nFeatures2D;
    readData(ifs, nFeatures2D);

    size_t nFeatures3D;
    readData(ifs, nFeatures3D);

    std::vector<FramePtr> frameMap(nFrames);
    for (size_t i = 0; i < nFrames; ++i)
    {
        frameMap.at(i) = boost::make_shared<Frame>();
    }

    std::vector<PosePtr> poseMap(nPoses);
    for (size_t i = 0; i < nPoses; ++i)
    {
        poseMap.at(i) = boost::make_shared<Pose>();
    }

    std::vector<OdometryPtr> odometryMap(nOdometry);
    for (size_t i = 0; i < nOdometry; ++i)
    {
        odometryMap.at(i) = boost::make_shared<Odometry>();
    }

    std::vector<Point2DFeaturePtr> feature2DMap(nFeatures2D);
    for (size_t i = 0; i < nFeatures2D; ++i)
    {
        feature2DMap.at(i) = boost::make_shared<Point2DFeature>();
    }

    std::vector<Point3DFeaturePtr> feature3DMap(nFeatures3D);
    for (size_t i = 0; i < nFeatures3D; ++i)
    {
        feature3DMap.at(i) = boost::make_shared<Point3DFeature>();
    }

    for (size_t i = 0; i < nFrames; ++i)
    {
        size_t frameId;
        readData(ifs, frameId);

        FramePtr& frame = frameMap.at(frameId);

        size_t imageFilenameLen;
        readData(ifs, imageFilenameLen);

        if (imageFilenameLen > 1)
        {
            char* imageFilename = new char[imageFilenameLen];
            ifs.read(imageFilename, imageFilenameLen);

            boost::filesystem::path imagePath = rootDir;
            imagePath /= imageFilename;

            frame->image() = cv::imread(imagePath.string().c_str(), -1);
            if (frame->image().empty())
            {
                std::cout << "# WARNING: Unable to read " << imagePath.string() << std::endl;
            }

            delete imageFilename;
        }

        readData(ifs, frame->cameraId());

        size_t poseId;
        readData(ifs, poseId);
        if (poseId != static_cast<size_t>(-1))
        {
            frame->cameraPose() = poseMap.at(poseId);
        }

        size_t odometryId;
        readData(ifs, odometryId);
        if (odometryId != static_cast<size_t>(-1))
        {
            frame->systemPose() = odometryMap.at(odometryId);
        }

        readData(ifs, odometryId);
        if (odometryId != static_cast<size_t>(-1))
        {
            frame->odometryMeasurement() = odometryMap.at(odometryId);
        }

        readData(ifs, poseId);
        if (poseId != static_cast<size_t>(-1))
        {
            frame->gpsInsMeasurement() = poseMap.at(poseId);
        }

        size_t nFeatures2D;
        readData(ifs, nFeatures2D);
        std::vector<Point2DFeaturePtr>& features2D = frame->features2D();
        features2D.resize(nFeatures2D);

        for (size_t j = 0; j < features2D.size(); ++j)
        {
            size_t feature2DId;
            readData(ifs, feature2DId);

            if (feature2DId != static_cast<size_t>(-1))
            {
                features2D.at(j) = feature2DMap.at(feature2DId);
            }
        }
    }

    for (size_t i = 0; i < nPoses; ++i)
    {
        size_t poseId;
        readData(ifs, poseId);

        PosePtr& pose = poseMap.at(poseId);

        readData(ifs, pose->timeStamp());

        double q[4];
        readData(ifs, q[0]);
        readData(ifs, q[1]);
        readData(ifs, q[2]);
        readData(ifs, q[3]);

        memcpy(pose->rotationData(), q, sizeof(double) * 4);

        double t[3];
        readData(ifs, t[0]);
        readData(ifs, t[1]);
        readData(ifs, t[2]);

        memcpy(pose->translationData(), t, sizeof(double) * 3);

        double cov[49];
        for (int j = 0; j < 49; ++j)
        {
            readData(ifs, cov[j]);
        }

        memcpy(pose->covarianceData(), cov, sizeof(double) * 49);
    }

    for (size_t i = 0; i < nOdometry; ++i)
    {
        size_t odometryId;
        readData(ifs, odometryId);

        OdometryPtr& odometry = odometryMap.at(odometryId);

        readData(ifs, odometry->timeStamp());
        readData(ifs, odometry->x());
        readData(ifs, odometry->y());
        readData(ifs, odometry->z());
        readData(ifs, odometry->yaw());
        readData(ifs, odometry->pitch());
        readData(ifs, odometry->roll());
    }

    std::vector<size_t> tmp;
    for (size_t i = 0; i < nFeatures2D; ++i)
    {
        size_t featureId;
        readData(ifs, featureId);

        tmp.push_back(featureId);

        Point2DFeaturePtr& feature2D = feature2DMap.at(featureId);

        int type, rows, cols;
        readData(ifs, type);
        readData(ifs, rows);
        readData(ifs, cols);

        feature2D->descriptor() = cv::Mat(rows, cols, type);

        cv::Mat& dtor = feature2D->descriptor();

        for (int r = 0; r < rows; ++r)
        {
            for (int c = 0; c < cols; ++c)
            {
            switch (dtor.type())
            {
                case CV_8U:
                    readData(ifs, dtor.at<unsigned char>(r,c));
                    break;
                case CV_8S:
                    readData(ifs, dtor.at<char>(r,c));
                    break;
                case CV_16U:
                    readData(ifs, dtor.at<unsigned short>(r,c));
                    break;
                case CV_16S:
                    readData(ifs, dtor.at<short>(r,c));
                    break;
                case CV_32S:
                    readData(ifs, dtor.at<int>(r,c));
                    break;
                case CV_32F:
                    readData(ifs, dtor.at<float>(r,c));
                    break;
                case CV_64F:
                default:
                    readData(ifs, dtor.at<double>(r,c));
                }
            }
        }

        readData(ifs, feature2D->keypoint().angle);
        readData(ifs, feature2D->keypoint().class_id);
        readData(ifs, feature2D->keypoint().octave);
        readData(ifs, feature2D->keypoint().pt.x);
        readData(ifs, feature2D->keypoint().pt.y);
        readData(ifs, feature2D->keypoint().response);
        readData(ifs, feature2D->keypoint().size);
        readData(ifs, feature2D->index());
        readData(ifs, feature2D->bestPrevMatchId());
        readData(ifs, feature2D->bestNextMatchId());

        size_t nPrevMatches;
        readData(ifs, nPrevMatches);
        feature2D->prevMatches().resize(nPrevMatches);

        for (size_t j = 0; j < feature2D->prevMatches().size(); ++j)
        {
            readData(ifs, featureId);

            if (featureId != static_cast<size_t>(-1))
            {
                feature2D->prevMatches().at(j) = feature2DMap.at(featureId);
            }
        }

        size_t nNextMatches;
        readData(ifs, nNextMatches);
        feature2D->nextMatches().resize(nNextMatches);

        for (size_t j = 0; j < feature2D->nextMatches().size(); ++j)
        {
            readData(ifs, featureId);

            if (featureId != static_cast<size_t>(-1))
            {
                feature2D->nextMatches().at(j) = feature2DMap.at(featureId);
            }
        }

        size_t feature3DId;
        readData(ifs, feature3DId);
        if (feature3DId != static_cast<size_t>(-1))
        {
            feature2D->feature3D() = feature3DMap.at(feature3DId);
        }

        size_t frameId;
        readData(ifs, frameId);
        if (frameId != static_cast<size_t>(-1))
        {
            feature2D->frame() = frameMap.at(frameId);
        }
    }

    for (size_t i = 0; i < nFeatures3D; ++i)
    {
        size_t featureId;
        readData(ifs, featureId);

        Point3DFeaturePtr& feature3D = feature3DMap.at(featureId);

        Eigen::Vector3d& P = feature3D->point();
        readData(ifs, P(0));
        readData(ifs, P(1));
        readData(ifs, P(2));

        double cov[9];
        for (int j = 0; j < 9; ++j)
        {
            readData(ifs, cov[j]);
        }

        memcpy(feature3D->pointCovarianceData(), cov, sizeof(double) * 9);

        readData(ifs, feature3D->attributes());
        readData(ifs, feature3D->weight());

        size_t nFeatures2D;
        readData(ifs, nFeatures2D);
        feature3D->features2D().resize(nFeatures2D);

        for (size_t j = 0; j < feature3D->features2D().size(); ++j)
        {
            readData(ifs, featureId);

            if (featureId != static_cast<size_t>(-1))
            {
                feature3D->features2D().at(j) = feature2DMap.at(featureId);
            }
        }
    }

    size_t nSegments;
    readData(ifs, nSegments);

    m_frameSetSegments.resize(nSegments);

    for (size_t segmentId = 0; segmentId < m_frameSetSegments.size(); ++segmentId)
    {
        size_t nFrameSets;
        readData(ifs, nFrameSets);
        m_frameSetSegments.at(segmentId).resize(nFrameSets);

        FrameSetSegment& segment = m_frameSetSegments.at(segmentId);

        for (size_t frameSetId = 0; frameSetId < segment.size(); ++frameSetId)
        {
            size_t frameSetSize;
            readData(ifs, frameSetSize);

            segment.at(frameSetId) = boost::make_shared<FrameSet>();
            FrameSetPtr& frameSet = segment.at(frameSetId);
            frameSet->frames().resize(frameSetSize);

            for (size_t i = 0; i < frameSet->frames().size(); ++i)
            {
                size_t frameId;
                readData(ifs, frameId);

                if (frameId != static_cast<size_t>(-1))
                {
                    frameSet->frames().at(i) = frameMap.at(frameId);
                }
            }

            size_t odometryId;
            readData(ifs, odometryId);
            if (odometryId != static_cast<size_t>(-1))
            {
                frameSet->systemPose() = odometryMap.at(odometryId);
            }

            readData(ifs, odometryId);
            if (odometryId != static_cast<size_t>(-1))
            {
                frameSet->odometryMeasurement() = odometryMap.at(odometryId);
            }

            size_t poseId;
            readData(ifs, poseId);
            if (poseId != static_cast<size_t>(-1))
            {
                frameSet->gpsInsMeasurement() = poseMap.at(poseId);
            }
        }
    }

    ifs.close();

    return true;
}

void
SparseGraph::writeToBinaryFile(const std::string& filename) const
{
    boost::filesystem::path filePath(filename);

    boost::filesystem::path imageDir;
    if (filePath.has_parent_path())
    {
        imageDir = filePath.parent_path();
        imageDir /= filePath.stem();
    }
    else
    {
        imageDir = filePath.stem();
    }

    // create image directory if it does not exist
    if (!boost::filesystem::exists(imageDir))
    {
        boost::filesystem::create_directory(imageDir);
    }

    // write frame data to binary file
    std::ofstream ofs(filename.c_str(), std::ios::out | std::ios::binary);
    if (!ofs.is_open())
    {
        return;
    }

    boost::unordered_map<Frame*,size_t> frameMap;
    boost::unordered_map<Pose*,size_t> poseMap;
    boost::unordered_map<Odometry*,size_t> odometryMap;
    boost::unordered_map<Point2DFeature*,size_t> feature2DMap;
    boost::unordered_map<Point3DFeature*,size_t> feature3DMap;

    for (size_t segmentId = 0; segmentId < m_frameSetSegments.size(); ++segmentId)
    {
        const FrameSetSegment& segment = m_frameSetSegments.at(segmentId);

        for (size_t frameSetId = 0; frameSetId < segment.size(); ++frameSetId)
        {
            const FrameSetPtr& frameSet = segment.at(frameSetId);

            // index all structures
            if (frameSet->systemPose().get() != 0)
            {
                if (odometryMap.find(frameSet->systemPose().get()) == odometryMap.end())
                {
                    odometryMap.insert(std::make_pair(frameSet->systemPose().get(), odometryMap.size()));
                }
            }
            if (frameSet->odometryMeasurement().get() != 0)
            {
                if (odometryMap.find(frameSet->odometryMeasurement().get()) == odometryMap.end())
                {
                    odometryMap.insert(std::make_pair(frameSet->odometryMeasurement().get(), odometryMap.size()));
                }
            }
            if (frameSet->gpsInsMeasurement().get() != 0)
            {
                if (poseMap.find(frameSet->gpsInsMeasurement().get()) == poseMap.end())
                {
                    poseMap.insert(std::make_pair(frameSet->gpsInsMeasurement().get(), poseMap.size()));
                }
            }

            for (size_t frameId = 0; frameId < frameSet->frames().size(); ++frameId)
            {
                const FramePtr& frame = frameSet->frames().at(frameId);

                if (frame.get() == 0)
                {
                    continue;
                }

                frameMap.insert(std::make_pair(frame.get(), frameMap.size()));

                if (frame->cameraPose().get() != 0)
                {
                    if (poseMap.find(frame->cameraPose().get()) == poseMap.end())
                    {
                        poseMap.insert(std::make_pair(frame->cameraPose().get(), poseMap.size()));
                    }
                }

                if (frame->systemPose().get() != 0)
                {
                    if (odometryMap.find(frame->systemPose().get()) == odometryMap.end())
                    {
                        odometryMap.insert(std::make_pair(frame->systemPose().get(), odometryMap.size()));
                    }
                }
                if (frame->odometryMeasurement().get() != 0)
                {
                    if (odometryMap.find(frame->odometryMeasurement().get()) == odometryMap.end())
                    {
                        odometryMap.insert(std::make_pair(frame->odometryMeasurement().get(), odometryMap.size()));
                    }
                }
                if (frame->gpsInsMeasurement().get() != 0)
                {
                    if (poseMap.find(frame->gpsInsMeasurement().get()) == poseMap.end())
                    {
                        poseMap.insert(std::make_pair(frame->gpsInsMeasurement().get(), poseMap.size()));
                    }
                }

                const std::vector<Point2DFeaturePtr>& features2D = frame->features2D();
                for (size_t i = 0; i < features2D.size(); ++i)
                {
                    const Point2DFeaturePtr& feature2D = features2D.at(i);
                    if (feature2D.get() == 0)
                    {
                        std::cout << "# WARNING: Frame::features2D: Empty Point2DFeaturePtr instance." << std::endl;
                        continue;
                    }

                    if (feature2DMap.find(feature2D.get()) == feature2DMap.end())
                    {
                        feature2DMap.insert(std::make_pair(feature2D.get(), feature2DMap.size()));
                    }

                    const Point3DFeaturePtr& feature3D = feature2D->feature3D();
                    if (feature3D.get() != 0)
                    {
                        if (feature3DMap.find(feature3D.get()) == feature3DMap.end())
                        {
                            feature3DMap.insert(std::make_pair(feature3D.get(), feature3DMap.size()));
                        }
                    }
                }
            }
        }
    }

    writeData(ofs, frameMap.size());
    writeData(ofs, poseMap.size());
    writeData(ofs, odometryMap.size());
    writeData(ofs, feature2DMap.size());
    writeData(ofs, feature3DMap.size());

    // link all references
    for (boost::unordered_map<Frame*,size_t>::iterator it = frameMap.begin();
             it != frameMap.end(); ++it)
    {
        Frame* frame = it->first;

        char frameName[255];
        sprintf(frameName, "frame%lu", it->second);

        writeData(ofs, it->second);

        // attributes
        if (!frame->image().empty())
        {
            char imageFilename[1024];
            sprintf(imageFilename, "%s/%s.png",
                    imageDir.string().c_str(), frameName);
            cv::imwrite(imageFilename, frame->image());

            memset(imageFilename, 0, 1024);
            sprintf(imageFilename, "%s/%s.png", filePath.stem().c_str(), frameName);

            size_t imageFilenameLen = strlen(imageFilename) + 1;
            writeData(ofs, imageFilenameLen);
            ofs.write(imageFilename, imageFilenameLen);
        }
        else
        {
            size_t emptyFilenameLen = 1;
            writeData(ofs, emptyFilenameLen);
        }

        writeData(ofs, frame->cameraId());

        // references
        if (frame->cameraPose().get() != 0)
        {
            writeData(ofs, poseMap[frame->cameraPose().get()]);
        }
        else
        {
            size_t invalidId = -1;
            writeData(ofs, invalidId);
        }

        if (frame->systemPose().get() != 0)
        {
            writeData(ofs, odometryMap[frame->systemPose().get()]);
        }
        else
        {
            size_t invalidId = -1;
            writeData(ofs, invalidId);
        }

        if (frame->odometryMeasurement().get() != 0)
        {
            writeData(ofs, odometryMap[frame->odometryMeasurement().get()]);
        }
        else
        {
            size_t invalidId = -1;
            writeData(ofs, invalidId);
        }

        if (frame->gpsInsMeasurement().get() != 0)
        {
            writeData(ofs, poseMap[frame->gpsInsMeasurement().get()]);
        }
        else
        {
            size_t invalidId = -1;
            writeData(ofs, invalidId);
        }

        writeData(ofs, frame->features2D().size());

        const std::vector<Point2DFeaturePtr>& features2D = frame->features2D();
        for (size_t i = 0; i < features2D.size(); ++i)
        {
            const Point2DFeaturePtr& feature2D = features2D.at(i);

            boost::unordered_map<Point2DFeature*,size_t>::iterator itF2D = feature2DMap.find(feature2D.get());
            if (itF2D != feature2DMap.end())
            {
                writeData(ofs, itF2D->second);
            }
            else
            {
                size_t invalidId = -1;
                writeData(ofs, invalidId);
            }
        }
    }

    for (boost::unordered_map<Pose*, size_t>::iterator it = poseMap.begin();
            it != poseMap.end(); ++it)
    {
        Pose* pose = it->first;

        writeData(ofs, it->second);
        writeData(ofs, pose->timeStamp());

        const double* const q = pose->rotationData();
        writeData(ofs, q[0]);
        writeData(ofs, q[1]);
        writeData(ofs, q[2]);
        writeData(ofs, q[3]);

        const double* const t = pose->translationData();
        writeData(ofs, t[0]);
        writeData(ofs, t[1]);
        writeData(ofs, t[2]);

        const double* const cov = pose->covarianceData();
        for (int i = 0; i < 49; ++i)
        {
            writeData(ofs, cov[i]);
        }
    }

    for (boost::unordered_map<Odometry*, size_t>::iterator it = odometryMap.begin();
            it != odometryMap.end(); ++it)
    {
        Odometry* odometry = it->first;

        writeData(ofs, it->second);

        writeData(ofs, odometry->timeStamp());
        writeData(ofs, odometry->x());
        writeData(ofs, odometry->y());
        writeData(ofs, odometry->z());
        writeData(ofs, odometry->yaw());
        writeData(ofs, odometry->pitch());
        writeData(ofs, odometry->roll());
    }

    for (boost::unordered_map<Point2DFeature*,size_t>::iterator it = feature2DMap.begin();
             it != feature2DMap.end(); ++it)
    {
        Point2DFeature* feature2D = it->first;

        writeData(ofs, it->second);

        // attributes
        const cv::Mat& dtor = feature2D->descriptor();

        writeData(ofs, dtor.type());
        writeData(ofs, dtor.rows);
        writeData(ofs, dtor.cols);

        for (int r = 0; r < dtor.rows; ++r)
        {
            for (int c = 0; c < dtor.cols; ++c)
            {
                switch (dtor.type())
                {
                case CV_8U:
                    writeData(ofs, dtor.at<unsigned char>(r,c));
                    break;
                case CV_8S:
                    writeData(ofs, dtor.at<char>(r,c));
                    break;
                case CV_16U:
                    writeData(ofs, dtor.at<unsigned short>(r,c));
                    break;
                case CV_16S:
                    writeData(ofs, dtor.at<short>(r,c));
                    break;
                case CV_32S:
                    writeData(ofs, dtor.at<int>(r,c));
                    break;
                case CV_32F:
                    writeData(ofs, dtor.at<float>(r,c));
                    break;
                case CV_64F:
                default:
                    writeData(ofs, dtor.at<double>(r,c));
                }
            }
        }

        writeData(ofs, feature2D->keypoint().angle);
        writeData(ofs, feature2D->keypoint().class_id);
        writeData(ofs, feature2D->keypoint().octave);
        writeData(ofs, feature2D->keypoint().pt.x);
        writeData(ofs, feature2D->keypoint().pt.y);
        writeData(ofs, feature2D->keypoint().response);
        writeData(ofs, feature2D->keypoint().size);
        writeData(ofs, feature2D->index());
        writeData(ofs, feature2D->bestPrevMatchId());
        writeData(ofs, feature2D->bestNextMatchId());

        // references
        writeData(ofs, feature2D->prevMatches().size());

        for (size_t i = 0; i < feature2D->prevMatches().size(); ++i)
        {
            bool valid = false;

            if (Point2DFeaturePtr prevMatch = feature2D->prevMatches().at(i).lock())
            {
                boost::unordered_map<Point2DFeature*,size_t>::iterator itF2D = feature2DMap.find(prevMatch.get());
                if (itF2D != feature2DMap.end())
                {
                    valid = true;
                    writeData(ofs, itF2D->second);
                }
            }

            if (!valid)
            {
                size_t invalidId = -1;
                writeData(ofs, invalidId);
            }
        }

        writeData(ofs, feature2D->nextMatches().size());

        for (size_t i = 0; i < feature2D->nextMatches().size(); ++i)
        {
            bool valid = false;

            if (Point2DFeaturePtr nextMatch = feature2D->nextMatches().at(i).lock())
            {
                boost::unordered_map<Point2DFeature*,size_t>::iterator itF2D = feature2DMap.find(nextMatch.get());
                if (itF2D != feature2DMap.end())
                {
                    valid = true;
                    writeData(ofs, itF2D->second);
                }
            }

            if (!valid)
            {
                size_t invalidId = -1;
                writeData(ofs, invalidId);
            }
        }

        if (feature2D->feature3D().get() != 0)
        {
            boost::unordered_map<Point3DFeature*,size_t>::iterator itF3D = feature3DMap.find(feature2D->feature3D().get());
            if (itF3D != feature3DMap.end())
            {
                writeData(ofs, itF3D->second);
            }
            else
            {
                size_t invalidId = -1;
                writeData(ofs, invalidId);
            }
        }
        else
        {
            size_t invalidId = -1;
            writeData(ofs, invalidId);
        }

        if (FramePtr frame = feature2D->frame().lock())
        {
            writeData(ofs, frameMap[frame.get()]);
        }
        else
        {
            size_t invalidId = -1;
            writeData(ofs, invalidId);
        }
    }

    for (boost::unordered_map<Point3DFeature*,size_t>::iterator it = feature3DMap.begin();
             it != feature3DMap.end(); ++it)
    {
        Point3DFeature* feature3D = it->first;

        // attributes
        writeData(ofs, it->second);

        const Eigen::Vector3d& P = feature3D->point();
        writeData(ofs, P(0));
        writeData(ofs, P(1));
        writeData(ofs, P(2));

        const double* const cov = feature3D->pointCovarianceData();
        for (int i = 0; i < 9; ++i)
        {
            writeData(ofs, cov[i]);
        }

        writeData(ofs, feature3D->attributes());
        writeData(ofs, feature3D->weight());

        // references
        writeData(ofs, feature3D->features2D().size());

        for (size_t i = 0; i < feature3D->features2D().size(); ++i)
        {
            bool valid = false;

            if (Point2DFeaturePtr feature2D = feature3D->features2D().at(i).lock())
            {
                boost::unordered_map<Point2DFeature*,size_t>::iterator itF2D = feature2DMap.find(feature2D.get());
                if (itF2D != feature2DMap.end())
                {
                    valid = true;
                    writeData(ofs, itF2D->second);
                }
            }

            if (!valid)
            {
                size_t invalidId = -1;
                writeData(ofs, invalidId);
            }
        }
    }

    writeData(ofs, m_frameSetSegments.size());

    for (size_t segmentId = 0; segmentId < m_frameSetSegments.size(); ++segmentId)
    {
        const FrameSetSegment& segment = m_frameSetSegments.at(segmentId);

        writeData(ofs, segment.size());

        for (size_t frameSetId = 0; frameSetId < segment.size(); ++frameSetId)
        {
            const FrameSetPtr& frameSet = segment.at(frameSetId);

            writeData(ofs, frameSet->frames().size());

            for (size_t frameId = 0; frameId < frameSet->frames().size(); ++frameId)
            {
                if (frameSet->frames().at(frameId).get() != 0)
                {
                    writeData(ofs, frameMap[frameSet->frames().at(frameId).get()]);
                }
                else
                {
                    size_t invalidId = -1;
                    writeData(ofs, invalidId);
                }
            }

            if (frameSet->systemPose().get() != 0)
            {
                writeData(ofs, odometryMap[frameSet->systemPose().get()]);
            }
            else
            {
                size_t invalidId = -1;
                writeData(ofs, invalidId);
            }

            if (frameSet->odometryMeasurement().get() != 0)
            {
                writeData(ofs, odometryMap[frameSet->odometryMeasurement().get()]);
            }
            else
            {
                size_t invalidId = -1;
                writeData(ofs, invalidId);
            }

            if (frameSet->gpsInsMeasurement().get() != 0)
            {
                writeData(ofs, poseMap[frameSet->gpsInsMeasurement().get()]);
            }
            else
            {
                size_t invalidId = -1;
                writeData(ofs, invalidId);
            }
        }
    }

    ofs.close();
}

template<typename T>
void
SparseGraph::readData(std::ifstream& ifs, T& data) const
{
    char* buffer = new char[sizeof(T)];

    ifs.read(buffer, sizeof(T));

    data = *(reinterpret_cast<T*>(buffer));

    delete [] buffer;
}

template<typename T>
void
SparseGraph::writeData(std::ofstream& ofs, T data) const
{
    char* pData = reinterpret_cast<char*>(&data);

    ofs.write(pData, sizeof(T));
}

}
