#include "camodocal/sparse_graph/SparseGraph.h"

#include <opencv2/highgui/highgui.hpp>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "../pugixml/pugixml.hpp"

namespace camodocal
{

Pose::Pose()
 : m_timeStamp(0)
{
    m_q.setIdentity();
    m_t.setZero();
    m_covariance.setZero();
}

Pose::Pose(const Eigen::Matrix4d& H)
: m_timeStamp(0)
{
   m_q = Eigen::Quaterniond(H.block<3,3>(0,0));
   m_t = H.block<3,1>(0,3);
}

uint64_t&
Pose::timeStamp(void)
{
    return m_timeStamp;
}

uint64_t
Pose::timeStamp(void) const
{
    return m_timeStamp;
}

Eigen::Quaterniond&
Pose::rotation(void)
{
    return m_q;
}

const Eigen::Quaterniond&
Pose::rotation(void) const
{
    return m_q;
}

double*
Pose::rotationData(void)
{
    return m_q.coeffs().data();
}

const double* const
Pose::rotationData(void) const
{
    return m_q.coeffs().data();
}

Eigen::Vector3d&
Pose::translation(void)
{
    return m_t;
}

const Eigen::Vector3d&
Pose::translation(void) const
{
    return m_t;
}

double*
Pose::translationData(void)
{
    return m_t.data();
}

const double* const
Pose::translationData(void) const
{
    return m_t.data();
}

Eigen::Matrix4d
Pose::pose(void) const
{
    Eigen::Matrix4d H;
    H.setIdentity();
    H.block<3,3>(0,0) = m_q.toRotationMatrix();
    H.block<3,1>(0,3) = m_t;

    return H;
}

Eigen::Matrix<double,7,7>&
Pose::covariance(void)
{
    return m_covariance;
}

const Eigen::Matrix<double,7,7>&
Pose::covariance(void) const
{
    return m_covariance;
}

double*
Pose::covarianceData(void)
{
    return m_covariance.data();
}

const double* const
Pose::covarianceData(void) const
{
    return m_covariance.data();
}

Frame::Frame()
 : m_cameraId(-1)
 , m_id(0)
{

}

PosePtr&
Frame::camera(void)
{
    return m_camera;
}

PoseConstPtr
Frame::camera(void) const
{
    return m_camera;
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
Frame::odometry(void)
{
    return m_odometry;
}

OdometryConstPtr
Frame::odometry(void) const
{
    return m_odometry;
}

PosePtr&
Frame::gps_ins(void)
{
    return m_gpsIns;
}

PoseConstPtr
Frame::gps_ins(void) const
{
    return m_gpsIns;
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

std::vector<Point3DFeaturePtr>&
Frame::features3D(void)
{
    return m_features3D;
}

const std::vector<Point3DFeaturePtr>&
Frame::features3D(void) const
{
    return m_features3D;
}

unsigned int&
Frame::id(void)
{
    return m_id;
}

unsigned int
Frame::id(void) const
{
    return m_id;
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
 , m_bestPrevMatchIdx(-1)
 , m_bestNextMatchIdx(-1)
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

Point2DFeaturePtr&
Point2DFeature::prevMatch(void)
{
    return m_prevMatches.at(m_bestPrevMatchIdx);
}

Point2DFeatureConstPtr
Point2DFeature::prevMatch(void) const
{
    return m_prevMatches.at(m_bestPrevMatchIdx);
}

std::vector<Point2DFeaturePtr>&
Point2DFeature::prevMatches(void)
{
    return m_prevMatches;
}

const std::vector<Point2DFeaturePtr>&
Point2DFeature::prevMatches(void) const
{
    return m_prevMatches;
}

int&
Point2DFeature::bestPrevMatchIdx(void)
{
    return m_bestPrevMatchIdx;
}

int
Point2DFeature::bestPrevMatchIdx(void) const
{
    return m_bestPrevMatchIdx;
}

Point2DFeaturePtr&
Point2DFeature::nextMatch(void)
{
    return m_nextMatches.at(m_bestNextMatchIdx);
}

Point2DFeatureConstPtr
Point2DFeature::nextMatch(void) const
{
    return m_nextMatches.at(m_bestNextMatchIdx);
}

std::vector<Point2DFeaturePtr>&
Point2DFeature::nextMatches(void)
{
    return m_nextMatches;
}

const std::vector<Point2DFeaturePtr>&
Point2DFeature::nextMatches(void) const
{
    return m_nextMatches;
}

int&
Point2DFeature::bestNextMatchIdx(void)
{
    return m_bestNextMatchIdx;
}

int
Point2DFeature::bestNextMatchIdx(void) const
{
    return m_bestNextMatchIdx;
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

FramePtr&
Point2DFeature::frame(void)
{
    return m_frame;
}

FrameConstPtr
Point2DFeature::frame(void) const
{
    return m_frame;
}

Point2DFeatureRightPtr&
Point2DFeatureLeft::rightCorrespondence(void)
{
    return m_rightCorrespondence;
}

Point2DFeatureRightConstPtr
Point2DFeatureLeft::rightCorrespondence(void) const
{
    return m_rightCorrespondence;
}

Point2DFeatureLeftPtr&
Point2DFeatureLeft::prevCorrespondence(void)
{
    return m_prevCorrespondence;
}

Point2DFeatureLeftConstPtr
Point2DFeatureLeft::prevCorrespondence(void) const
{
    return m_prevCorrespondence;
}

Point2DFeatureLeftPtr&
Point2DFeatureRight::leftCorrespondence(void)
{
    return m_leftCorrespondence;
}

Point2DFeatureLeftConstPtr
Point2DFeatureRight::leftCorrespondence(void) const
{
    return m_leftCorrespondence;
}

Point2DFeatureRightPtr&
Point2DFeatureRight::prevCorrespondence(void)
{
    return m_prevCorrespondence;
}

Point2DFeatureRightConstPtr
Point2DFeatureRight::prevCorrespondence(void) const
{
    return m_prevCorrespondence;
}

Point3DFeature::Point3DFeature(void)
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

std::vector<Point2DFeaturePtr>&
Point3DFeature::features2D(void)
{
    return m_features2D;
}

const std::vector<Point2DFeaturePtr>&
Point3DFeature::features2D(void) const
{
    return m_features2D;
}

SparseGraph::SparseGraph()
{

}

int
SparseGraph::cameraCount(void) const
{
    return m_frameSegments.size();
}

std::vector<FrameSegment>&
SparseGraph::frameSegments(int cameraIdx)
{
    if (cameraIdx >= m_frameSegments.size())
    {
        m_frameSegments.resize(cameraIdx + 1);
    }

    return m_frameSegments.at(cameraIdx);
}

const std::vector<FrameSegment>&
SparseGraph::frameSegments(int cameraIdx) const
{
    return m_frameSegments.at(cameraIdx);
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

    m_frameSegments.clear();

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

    FrameSegment frameMap(nFrames);
    for (size_t i = 0; i < nFrames; ++i)
    {
        frameMap.at(i) = FramePtr(new Frame);
    }

    std::vector<PosePtr> poseMap(nPoses);
    for (size_t i = 0; i < nPoses; ++i)
    {
        poseMap.at(i) = PosePtr(new Pose);
    }

    std::vector<OdometryPtr> odometryMap(nOdometry);
    for (size_t i = 0; i < nOdometry; ++i)
    {
        odometryMap.at(i) = OdometryPtr(new Odometry);
    }

    std::vector<Point2DFeaturePtr> feature2DMap(nFeatures2D);
    for (size_t i = 0; i < nFeatures2D; ++i)
    {
        feature2DMap.at(i) = Point2DFeaturePtr(new Point2DFeature);
    }

    std::vector<Point3DFeaturePtr> feature3DMap(nFeatures3D);
    for (size_t i = 0; i < nFeatures3D; ++i)
    {
        feature3DMap.at(i) = Point3DFeaturePtr(new Point3DFeature);
    }

    for (size_t i = 0; i < nFrames; ++i)
    {
        size_t frameIdx;
        readData(ifs, frameIdx);

        FramePtr& frame = frameMap.at(frameIdx);

        size_t imageFilenameLen;
        readData(ifs, imageFilenameLen);

        if (imageFilenameLen > 1)
        {
            char* imageFilename = new char[imageFilenameLen];
            ifs.read(imageFilename, imageFilenameLen);

            boost::filesystem::path imagePath = rootDir;
            imagePath /= imageFilename;

            frame->image() = cv::imread(imagePath.string().c_str(), -1);

            delete imageFilename;
        }

        readData(ifs, frame->id());

        size_t poseIdx;
        readData(ifs, poseIdx);
        if (poseIdx != static_cast<size_t>(-1))
        {
            frame->camera() = poseMap.at(poseIdx);
        }

        size_t odometryIdx;
        readData(ifs, odometryIdx);
        if (odometryIdx != static_cast<size_t>(-1))
        {
            frame->odometry() = odometryMap.at(odometryIdx);
        }

        readData(ifs, poseIdx);
        if (poseIdx != static_cast<size_t>(-1))
        {
            frame->gps_ins() = poseMap.at(poseIdx);
        }

        size_t nFeatures2D;
        readData(ifs, nFeatures2D);
        std::vector<Point2DFeaturePtr>& features2D = frame->features2D();
        features2D.resize(nFeatures2D);

        for (size_t j = 0; j < features2D.size(); ++j)
        {
            size_t feature2DIdx;
            readData(ifs, feature2DIdx);

            features2D.at(j) = feature2DMap.at(feature2DIdx);
        }

        size_t nFeatures3D;
        readData(ifs, nFeatures3D);
        std::vector<Point3DFeaturePtr>& features3D = frame->features3D();
        features3D.resize(nFeatures3D);

        for (size_t j = 0; j < features3D.size(); ++j)
        {
            size_t feature3DIdx;
            readData(ifs, feature3DIdx);

            features3D.at(j) = feature3DMap.at(feature3DIdx);
        }
    }

    for (size_t i = 0; i < nPoses; ++i)
    {
        size_t poseIdx;
        readData(ifs, poseIdx);

        PosePtr& pose = poseMap.at(poseIdx);

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
        size_t odometryIdx;
        readData(ifs, odometryIdx);

        OdometryPtr& odometry = odometryMap.at(odometryIdx);

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
        size_t featureIdx;
        readData(ifs, featureIdx);

        tmp.push_back(featureIdx);

        Point2DFeaturePtr& feature2D = feature2DMap.at(featureIdx);

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
        readData(ifs, feature2D->bestPrevMatchIdx());
        readData(ifs, feature2D->bestNextMatchIdx());

        size_t nPrevMatches;
        readData(ifs, nPrevMatches);
        feature2D->prevMatches().resize(nPrevMatches);

        for (size_t j = 0; j < feature2D->prevMatches().size(); ++j)
        {
            readData(ifs, featureIdx);

            feature2D->prevMatches().at(j) = feature2DMap.at(featureIdx);
        }

        size_t nNextMatches;
        readData(ifs, nNextMatches);
        feature2D->nextMatches().resize(nNextMatches);

        for (size_t j = 0; j < feature2D->nextMatches().size(); ++j)
        {
            readData(ifs, featureIdx);

            feature2D->nextMatches().at(j) = feature2DMap.at(featureIdx);
        }

        size_t feature3DIdx;
        readData(ifs, feature3DIdx);
        if (feature3DIdx != static_cast<size_t>(-1))
        {
            feature2D->feature3D() = feature3DMap.at(feature3DIdx);
        }

        size_t frameIdx;
        readData(ifs, frameIdx);
        if (frameIdx != static_cast<size_t>(-1))
        {
            feature2D->frame() = frameMap.at(frameIdx);
        }
    }

    for (size_t i = 0; i < nFeatures3D; ++i)
    {
        size_t featureIdx;
        readData(ifs, featureIdx);

        Point3DFeaturePtr& feature3D = feature3DMap.at(featureIdx);

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

        size_t nFeatures2D;
        readData(ifs, nFeatures2D);
        feature3D->features2D().resize(nFeatures2D);

        for (size_t j = 0; j < feature3D->features2D().size(); ++j)
        {
            readData(ifs, featureIdx);

            feature3D->features2D().at(j) = feature2DMap.at(featureIdx);
        }
    }

    size_t nCameras;
    readData(ifs, nCameras);

    m_frameSegments.resize(nCameras);

    size_t frameCount = 0;
    for (size_t cameraIdx = 0; cameraIdx < m_frameSegments.size(); ++cameraIdx)
    {
        size_t nSegments;
        readData(ifs, nSegments);
        m_frameSegments.at(cameraIdx).resize(nSegments);

        for (size_t segmentIdx = 0; segmentIdx < m_frameSegments.at(cameraIdx).size(); ++segmentIdx)
        {
            size_t nFramesSegment;
            readData(ifs, nFramesSegment);

            FrameSegment& segment = m_frameSegments.at(cameraIdx).at(segmentIdx);
            segment.resize(nFramesSegment);

            for (size_t i = 0; i < segment.size(); ++i)
            {
                segment.at(i) = frameMap.at(frameCount);

                segment.at(i)->cameraId() = cameraIdx;

                ++frameCount;
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
        imageDir /= "images";
    }
    else
    {
        imageDir = boost::filesystem::path("images");
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

    for (size_t cameraIdx = 0; cameraIdx < m_frameSegments.size(); ++cameraIdx)
    {
        for (size_t segmentIdx = 0; segmentIdx < m_frameSegments.at(cameraIdx).size(); ++segmentIdx)
        {
            const FrameSegment& segment = m_frameSegments.at(cameraIdx).at(segmentIdx);

            // index all structures
            for (size_t frameIdx = 0; frameIdx < segment.size(); ++frameIdx)
            {
                const FramePtr& frame = segment.at(frameIdx);

                frameMap.insert(std::make_pair(frame.get(), frameMap.size()));

                if (frame->camera().get() != 0)
                {
                    if (poseMap.find(frame->camera().get()) == poseMap.end())
                    {
                        poseMap.insert(std::make_pair(frame->camera().get(), poseMap.size()));
                    }
                }
                if (frame->odometry().get() != 0)
                {
                    if (odometryMap.find(frame->odometry().get()) == odometryMap.end())
                    {
                        odometryMap.insert(std::make_pair(frame->odometry().get(), odometryMap.size()));
                    }
                }
                if (frame->gps_ins().get() != 0)
                {
                    if (poseMap.find(frame->gps_ins().get()) == poseMap.end())
                    {
                        poseMap.insert(std::make_pair(frame->gps_ins().get(), poseMap.size()));
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

                const std::vector<Point3DFeaturePtr>& features3D = frame->features3D();
                for (size_t i = 0; i < features3D.size(); ++i)
                {
                    const Point3DFeaturePtr& feature3D = features3D.at(i);
                    if (feature3D.get() == 0)
                    {
                        std::cout << "# WARNING: Frame::features3D: Empty Point3DFeaturePtr instance." << std::endl;
                        continue;
                    }

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
            sprintf(imageFilename, "images/%s.png", frameName);

            size_t imageFilenameLen = strlen(imageFilename) + 1;
            writeData(ofs, imageFilenameLen);
            ofs.write(imageFilename, imageFilenameLen);
        }
        else
        {
            size_t emptyFilenameLen = 1;
            writeData(ofs, emptyFilenameLen);
        }

        writeData(ofs, frame->id());

        // references
        if (frame->camera().get() != 0)
        {
            writeData(ofs, poseMap[frame->camera().get()]);
        }
        else
        {
            size_t invalidIdx = -1;
            writeData(ofs, invalidIdx);
        }

        if (frame->odometry().get() != 0)
        {
            writeData(ofs, odometryMap[frame->odometry().get()]);
        }
        else
        {
            size_t invalidIdx = -1;
            writeData(ofs, invalidIdx);
        }

        if (frame->gps_ins().get() != 0)
        {
            writeData(ofs, poseMap[frame->gps_ins().get()]);
        }
        else
        {
            size_t invalidIdx = -1;
            writeData(ofs, invalidIdx);
        }

        writeData(ofs, frame->features2D().size());

        const std::vector<Point2DFeaturePtr>& features2D = frame->features2D();
        for (size_t i = 0; i < features2D.size(); ++i)
        {
            const Point2DFeaturePtr& feature2D = features2D.at(i);

            boost::unordered_map<Point2DFeature*,size_t>::iterator itF2D = feature2DMap.find(feature2D.get());
            if (itF2D == feature2DMap.end())
            {
                continue;
            }

            writeData(ofs, itF2D->second);
        }

        writeData(ofs, frame->features3D().size());

        const std::vector<Point3DFeaturePtr>& features3D = frame->features3D();
        for (size_t i = 0; i < features3D.size(); ++i)
        {
            const Point3DFeaturePtr& feature3D = features3D.at(i);

            boost::unordered_map<Point3DFeature*,size_t>::iterator itF3D = feature3DMap.find(feature3D.get());
            if (itF3D == feature3DMap.end())
            {
                continue;
            }

            writeData(ofs, itF3D->second);
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
        writeData(ofs, feature2D->bestPrevMatchIdx());
        writeData(ofs, feature2D->bestNextMatchIdx());

        // references
        size_t nValidPrevMatches = 0;
        for (size_t i = 0; i < feature2D->prevMatches().size(); ++i)
        {
            boost::unordered_map<Point2DFeature*,size_t>::iterator itF2D = feature2DMap.find(feature2D->prevMatches().at(i).get());
            if (itF2D == feature2DMap.end())
            {
//                std::cout << "# WARNING: Feature2D::prevMatches: Point2DFeature instance was not found in map.\n";
                continue;
            }

            ++nValidPrevMatches;
        }

        writeData(ofs, nValidPrevMatches);

        for (size_t i = 0; i < feature2D->prevMatches().size(); ++i)
        {
            boost::unordered_map<Point2DFeature*,size_t>::iterator itF2D = feature2DMap.find(feature2D->prevMatches().at(i).get());
            if (itF2D == feature2DMap.end())
            {
//                std::cout << "# WARNING: Feature2D::prevMatches: Point2DFeature instance was not found in map.\n";
                continue;
            }

            writeData(ofs, itF2D->second);
        }

        size_t nValidNextMatches = 0;
        for (size_t i = 0; i < feature2D->nextMatches().size(); ++i)
        {
            boost::unordered_map<Point2DFeature*,size_t>::iterator itF2D = feature2DMap.find(feature2D->nextMatches().at(i).get());
            if (itF2D == feature2DMap.end())
            {
//                std::cout << "# WARNING: Feature2D::nextMatches: Point2DFeature instance was not found in map.\n";
                continue;
            }

            ++nValidNextMatches;
        }

        writeData(ofs, nValidNextMatches);

        for (size_t i = 0; i < feature2D->nextMatches().size(); ++i)
        {
            boost::unordered_map<Point2DFeature*,size_t>::iterator itF2D = feature2DMap.find(feature2D->nextMatches().at(i).get());
            if (itF2D == feature2DMap.end())
            {
//                std::cout << "# WARNING: Feature2D::nextMatches: Point2DFeature instance was not found in map.\n";
                continue;
            }

            writeData(ofs, itF2D->second);
        }

        if (feature2D->feature3D().get() != 0)
        {
            boost::unordered_map<Point3DFeature*,size_t>::iterator itF3D = feature3DMap.find(feature2D->feature3D().get());
            if (itF3D == feature3DMap.end())
            {
//                std::cout << "# WARNING: Feature2D::feature3D: Point3DFeature instance was not found in map.\n";
                continue;
            }

            writeData(ofs, itF3D->second);
        }
        else
        {
            size_t invalidIdx = -1;
            writeData(ofs, invalidIdx);
        }

        if (feature2D->frame().get() != 0)
        {
            writeData(ofs, frameMap[feature2D->frame().get()]);
        }
        else
        {
            size_t invalidIdx = -1;
            writeData(ofs, invalidIdx);
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

        // references
        size_t nValidFeatures2D = 0;
        for (size_t i = 0; i < feature3D->features2D().size(); ++i)
        {
            boost::unordered_map<Point2DFeature*,size_t>::iterator itF2D = feature2DMap.find(feature3D->features2D().at(i).get());
            if (itF2D == feature2DMap.end())
            {
//                std::cout << "# WARNING: Feature3D::features2D: Point2DFeature instance was not found in map.\n";
                continue;
            }

            ++nValidFeatures2D;
        }

        writeData(ofs, nValidFeatures2D);

        for (size_t i = 0; i < feature3D->features2D().size(); ++i)
        {
            boost::unordered_map<Point2DFeature*,size_t>::iterator itF2D = feature2DMap.find(feature3D->features2D().at(i).get());
            if (itF2D == feature2DMap.end())
            {
//                std::cout << "# WARNING: Feature3D::features2D: Point2DFeature instance was not found in map.\n";
                continue;
            }

            writeData(ofs, itF2D->second);
        }
    }

    writeData(ofs, m_frameSegments.size());

    size_t frameCount = 0;
    for (size_t cameraIdx = 0; cameraIdx < m_frameSegments.size(); ++cameraIdx)
    {
        writeData(ofs, m_frameSegments.at(cameraIdx).size());

        for (size_t segmentIdx = 0; segmentIdx < m_frameSegments.at(cameraIdx).size(); ++segmentIdx)
        {
            const FrameSegment& segment = m_frameSegments.at(cameraIdx).at(segmentIdx);

            writeData(ofs, segment.size());
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

    delete buffer;
}

template<typename T>
void
SparseGraph::writeData(std::ofstream& ofs, T data) const
{
    char* pData = reinterpret_cast<char*>(&data);

    ofs.write(pData, sizeof(T));
}

}
