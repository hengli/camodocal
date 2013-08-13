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
 : mTimeStamp(0)
{
    m_q.setIdentity();
    m_t.setZero();
    m_covariance.setZero();
}

Pose::Pose(const Eigen::Matrix4d& H)
: mTimeStamp(0)
{
   m_q = Eigen::Quaterniond(H.block<3,3>(0,0));
   m_t = H.block<3,1>(0,3);
}

uint64_t&
Pose::timeStamp(void)
{
    return mTimeStamp;
}

uint64_t
Pose::timeStamp(void) const
{
    return mTimeStamp;
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

Frame::Frame()
 : mCameraId(-1)
 , mId(0)
{

}

PosePtr&
Frame::camera(void)
{
    return mCamera;
}

PoseConstPtr
Frame::camera(void) const
{
    return mCamera;
}

int&
Frame::cameraId(void)
{
    return mCameraId;
}

int
Frame::cameraId(void) const
{
    return mCameraId;
}

OdometryPtr&
Frame::odometry(void)
{
    return mOdometry;
}

OdometryConstPtr
Frame::odometry(void) const
{
    return mOdometry;
}

PosePtr&
Frame::gps_ins(void)
{
    return mGpsIns;
}

PoseConstPtr
Frame::gps_ins(void) const
{
    return mGpsIns;
}

std::vector<Point2DFeaturePtr>&
Frame::features2D(void)
{
    return mFeatures2D;
}

const std::vector<Point2DFeaturePtr>&
Frame::features2D(void) const
{
    return mFeatures2D;
}

std::vector<Point3DFeaturePtr>&
Frame::features3D(void)
{
    return mFeatures3D;
}

const std::vector<Point3DFeaturePtr>&
Frame::features3D(void) const
{
    return mFeatures3D;
}

unsigned int&
Frame::id(void)
{
    return mId;
}

unsigned int
Frame::id(void) const
{
    return mId;
}

cv::Mat&
Frame::image(void)
{
    return mImage;
}

const cv::Mat&
Frame::image(void) const
{
    return mImage;
}

Point2DFeature::Point2DFeature()
 : mIndex(0)
 , mBestPrevMatchIdx(-1)
 , mBestNextMatchIdx(-1)
{

}

cv::Mat&
Point2DFeature::descriptor(void)
{
    return mDtor;
}

const cv::Mat&
Point2DFeature::descriptor(void) const
{
    return mDtor;
}

cv::KeyPoint&
Point2DFeature::keypoint(void)
{
    return mKeypoint;
}

const cv::KeyPoint&
Point2DFeature::keypoint(void) const
{
    return mKeypoint;
}

unsigned int&
Point2DFeature::index(void)
{
    return mIndex;
}

unsigned int
Point2DFeature::index(void) const
{
    return mIndex;
}

Point2DFeaturePtr&
Point2DFeature::prevMatch(void)
{
    return mPrevMatches.at(mBestPrevMatchIdx);
}

Point2DFeatureConstPtr
Point2DFeature::prevMatch(void) const
{
    return mPrevMatches.at(mBestPrevMatchIdx);
}

std::vector<Point2DFeaturePtr>&
Point2DFeature::prevMatches(void)
{
    return mPrevMatches;
}

const std::vector<Point2DFeaturePtr>&
Point2DFeature::prevMatches(void) const
{
    return mPrevMatches;
}

int&
Point2DFeature::bestPrevMatchIdx(void)
{
    return mBestPrevMatchIdx;
}

int
Point2DFeature::bestPrevMatchIdx(void) const
{
    return mBestPrevMatchIdx;
}

Point2DFeaturePtr&
Point2DFeature::nextMatch(void)
{
    return mNextMatches.at(mBestNextMatchIdx);
}

Point2DFeatureConstPtr
Point2DFeature::nextMatch(void) const
{
    return mNextMatches.at(mBestNextMatchIdx);
}

std::vector<Point2DFeaturePtr>&
Point2DFeature::nextMatches(void)
{
    return mNextMatches;
}

const std::vector<Point2DFeaturePtr>&
Point2DFeature::nextMatches(void) const
{
    return mNextMatches;
}

int&
Point2DFeature::bestNextMatchIdx(void)
{
    return mBestNextMatchIdx;
}

int
Point2DFeature::bestNextMatchIdx(void) const
{
    return mBestNextMatchIdx;
}

Point3DFeaturePtr&
Point2DFeature::feature3D(void)
{
    return mFeature3D;
}

Point3DFeatureConstPtr
Point2DFeature::feature3D(void) const
{
    return mFeature3D;
}

FramePtr&
Point2DFeature::frame(void)
{
    return mFrame;
}

FrameConstPtr
Point2DFeature::frame(void) const
{
    return mFrame;
}

Point2DFeatureRightPtr&
Point2DFeatureLeft::rightCorrespondence(void)
{
    return mRightCorrespondence;
}

Point2DFeatureRightConstPtr
Point2DFeatureLeft::rightCorrespondence(void) const
{
    return mRightCorrespondence;
}

Point2DFeatureLeftPtr&
Point2DFeatureLeft::prevCorrespondence(void)
{
    return mPrevCorrespondence;
}

Point2DFeatureLeftConstPtr
Point2DFeatureLeft::prevCorrespondence(void) const
{
    return mPrevCorrespondence;
}

Point2DFeatureLeftPtr&
Point2DFeatureRight::leftCorrespondence(void)
{
    return mLeftCorrespondence;
}

Point2DFeatureLeftConstPtr
Point2DFeatureRight::leftCorrespondence(void) const
{
    return mLeftCorrespondence;
}

Point2DFeatureRightPtr&
Point2DFeatureRight::prevCorrespondence(void)
{
    return mPrevCorrespondence;
}

Point2DFeatureRightConstPtr
Point2DFeatureRight::prevCorrespondence(void) const
{
    return mPrevCorrespondence;
}

Point3DFeature::Point3DFeature(void)
{
    mPoint.setZero();
}

Eigen::Vector3d&
Point3DFeature::point(void)
{
    return mPoint;
}

const Eigen::Vector3d&
Point3DFeature::point(void) const
{
    return mPoint;
}

double*
Point3DFeature::pointData(void)
{
    return mPoint.data();
}

const double* const
Point3DFeature::pointData(void) const
{
    return mPoint.data();
}

std::vector<Point2DFeaturePtr>&
Point3DFeature::features2D(void)
{
    return mFeatures2D;
}

const std::vector<Point2DFeaturePtr>&
Point3DFeature::features2D(void) const
{
    return mFeatures2D;
}

SparseGraph::SparseGraph()
{

}

int
SparseGraph::cameraCount(void) const
{
    return mFrameSegments.size();
}

std::vector<FrameSegment>&
SparseGraph::frameSegments(int cameraIdx)
{
    if (cameraIdx >= mFrameSegments.size())
    {
        mFrameSegments.resize(cameraIdx + 1);
    }

    return mFrameSegments.at(cameraIdx);
}

const std::vector<FrameSegment>&
SparseGraph::frameSegments(int cameraIdx) const
{
    return mFrameSegments.at(cameraIdx);
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

    mFrameSegments.clear();

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

    mFrameSegments.resize(nCameras);

    size_t frameCount = 0;
    for (size_t cameraIdx = 0; cameraIdx < mFrameSegments.size(); ++cameraIdx)
    {
        size_t nSegments;
        readData(ifs, nSegments);
        mFrameSegments.at(cameraIdx).resize(nSegments);

        for (size_t segmentIdx = 0; segmentIdx < mFrameSegments.at(cameraIdx).size(); ++segmentIdx)
        {
            size_t nFramesSegment;
            readData(ifs, nFramesSegment);

            FrameSegment& segment = mFrameSegments.at(cameraIdx).at(segmentIdx);
            segment.resize(nFramesSegment);

            for (size_t i = 0; i < segment.size(); ++i)
            {
                segment.at(i) = frameMap.at(frameCount);

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

    for (size_t cameraIdx = 0; cameraIdx < mFrameSegments.size(); ++cameraIdx)
    {
        for (size_t segmentIdx = 0; segmentIdx < mFrameSegments.at(cameraIdx).size(); ++segmentIdx)
        {
            const FrameSegment& segment = mFrameSegments.at(cameraIdx).at(segmentIdx);

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

    writeData(ofs, mFrameSegments.size());

    size_t frameCount = 0;
    for (size_t cameraIdx = 0; cameraIdx < mFrameSegments.size(); ++cameraIdx)
    {
        writeData(ofs, mFrameSegments.at(cameraIdx).size());

        for (size_t segmentIdx = 0; segmentIdx < mFrameSegments.at(cameraIdx).size(); ++segmentIdx)
        {
            const FrameSegment& segment = mFrameSegments.at(cameraIdx).at(segmentIdx);

            writeData(ofs, segment.size());
        }
    }

    ofs.close();
}

bool
SparseGraph::readFromXMLFile(const std::string& filename)
{
    boost::filesystem::path xmlPath(filename);

    boost::filesystem::path rootDir;
    if (xmlPath.has_parent_path())
    {
        rootDir = xmlPath.parent_path();
    }
    else
    {
        rootDir = boost::filesystem::path(".");
    }

    mFrameSegments.clear();

    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(filename.c_str());
    if (!result)
    {
        std::cout << "# WARNING: Unable to open/parse " << filename << ". "
                  << result.description() << "." << std::endl;
        return false;
    }

    // parse xml file

    // odometry and 3D feature structures are in global scope as
    // multiple frames from multiple cameras reference these structures
    pugi::xml_node eRoot = doc.child("root");
    unsigned int nCameras = eRoot.attribute("cameras_size").as_uint();

    mFrameSegments.resize(nCameras);

    FrameSegment frameMap;
    std::vector<pugi::xml_node> frameXMLMap;
    std::vector<PosePtr> poseMap;
    std::vector<OdometryPtr> odometryMap;
    std::vector<Point2DFeaturePtr> feature2DMap;
    std::vector<pugi::xml_node> feature2DXMLMap;
    std::vector<Point3DFeaturePtr> feature3DMap;
    std::vector<pugi::xml_node> feature3DXMLMap;

    pugi::xml_node eFrames = eRoot.child("frames");
    pugi::xml_node ePoses = eRoot.child("poses");
    pugi::xml_node eOdometry = eRoot.child("odometry");
    pugi::xml_node eFeatures2D = eRoot.child("features2D");
    pugi::xml_node eFeatures3D = eRoot.child("features3D");

    unsigned int nFrames = eFrames.attribute("size").as_uint();
    XMLToFrames(eFrames, nFrames, frameMap, frameXMLMap, rootDir);

    unsigned int nPoses = ePoses.attribute("size").as_uint();
    XMLToPoses(ePoses, nPoses, poseMap);

    unsigned int nOdometry = eOdometry.attribute("size").as_uint();
    XMLToOdometry(eOdometry, nOdometry, odometryMap);

    unsigned int nFeatures2D = eFeatures2D.attribute("size").as_uint();
    XMLToPoint2DFeatures(eFeatures2D, nFeatures2D, feature2DMap, feature2DXMLMap);

    unsigned int nFeatures3D = eFeatures3D.attribute("size").as_uint();
    XMLToPoint3DFeatures(eFeatures3D, nFeatures3D, feature3DMap, feature3DXMLMap);

    for (size_t cameraIdx = 0; cameraIdx < mFrameSegments.size(); ++cameraIdx)
    {
        char cameraName[255];
        sprintf(cameraName, "camera%lu", cameraIdx);
        pugi::xml_node eCamera = eRoot.child(cameraName);

        unsigned int nSegments = eCamera.attribute("segments_size").as_uint();
        mFrameSegments.at(cameraIdx).resize(nSegments);

        for (size_t segmentIdx = 0; segmentIdx < mFrameSegments.at(cameraIdx).size(); ++segmentIdx)
        {
            char segmentName[255];
            sprintf(segmentName, "segment%lu", segmentIdx);
            pugi::xml_node eSegment = eCamera.child(segmentName);

            unsigned int nFramesSegment = eSegment.attribute("frames_size").as_uint();

            FrameSegment& segment = mFrameSegments.at(cameraIdx).at(segmentIdx);
            segment.resize(nFramesSegment);

            pugi::xml_attribute aFramesSegment = eSegment.child("frames").first_attribute();
            for (size_t i = 0; i < segment.size(); ++i)
            {
                unsigned int key;
                sscanf(aFramesSegment.name(), "frame%u", &key);

                unsigned int value;
                sscanf(aFramesSegment.value(), "frame%u", &value);

                segment.at(key) = frameMap.at(value);

                aFramesSegment = aFramesSegment.next_attribute();
            }
        }
    }

    // link all references
    for (size_t i = 0; i < frameXMLMap.size(); ++i)
    {
        FramePtr& frame = frameMap.at(i);
        pugi::xml_node eFrame = frameXMLMap.at(i);

        pugi::xml_attribute aCamera = eFrame.attribute("camera");
        if (!aCamera.empty())
        {
            size_t poseIdx;
            sscanf(aCamera.value(), "pose%lu", &poseIdx);

            frame->camera() = poseMap.at(poseIdx);
        }

        pugi::xml_attribute aOdometry = eFrame.attribute("odometry");
        if (!aOdometry.empty())
        {
            size_t odometryIdx;
            sscanf(aOdometry.value(), "odometry%lu", &odometryIdx);

            frame->odometry() = odometryMap.at(odometryIdx);
        }

        pugi::xml_attribute aGpsIns = eFrame.attribute("gps_ins");
        if (!aGpsIns.empty())
        {
            size_t poseIdx;
            sscanf(aGpsIns.value(), "pose%lu", &poseIdx);

            frame->gps_ins() = poseMap.at(poseIdx);
        }

        unsigned int nFeatures2D = eFrame.attribute("features2D_size").as_uint();
        std::vector<Point2DFeaturePtr>& features2D = frame->features2D();
        features2D.resize(nFeatures2D);

        pugi::xml_attribute aFeatures2D = eFrame.child("features2D").first_attribute();
        for (size_t i = 0; i < features2D.size(); ++i)
        {
            unsigned int key;
            sscanf(aFeatures2D.name(), "features2D_%u", &key);

            unsigned int value;
            sscanf(aFeatures2D.value(), "F2D-%u", &value);

            features2D.at(key) = feature2DMap.at(value);

            aFeatures2D = aFeatures2D.next_attribute();
        }

        unsigned int nFeatures3D = eFrame.attribute("features3D_size").as_uint();
        std::vector<Point3DFeaturePtr>& features3D = frame->features3D();
        features3D.resize(nFeatures3D);

        pugi::xml_attribute aFeatures3D = eFrame.child("features3D").first_attribute();
        for (size_t i = 0; i < features3D.size(); ++i)
        {
            unsigned int key;
            sscanf(aFeatures3D.name(), "features3D_%u", &key);

            unsigned int value;
            sscanf(aFeatures3D.value(), "F3D-%u", &value);

            features3D.at(key) = feature3DMap.at(value);

            aFeatures3D = aFeatures3D.next_attribute();
        }
    }

    for (size_t i = 0; i < feature2DXMLMap.size(); ++i)
    {
        Point2DFeaturePtr& feature2D = feature2DMap.at(i);
        pugi::xml_node eFeature2D = feature2DXMLMap.at(i);

        unsigned int nPrevMatches = eFeature2D.attribute("prev_matches_size").as_uint();
        feature2D->prevMatches().resize(nPrevMatches);

        pugi::xml_attribute aFeature2DPrevMatches = eFeature2D.child("prev_matches").first_attribute();
        for (size_t j = 0; j < feature2D->prevMatches().size(); ++j)
        {
            unsigned int key;
            sscanf(aFeature2DPrevMatches.name(), "prev_matches_%u", &key);

            unsigned int value;
            sscanf(aFeature2DPrevMatches.value(), "F2D-%u", &value);

            feature2D->prevMatches().at(key) = feature2DMap.at(value);

            aFeature2DPrevMatches = aFeature2DPrevMatches.next_attribute();
        }

        unsigned int nNextMatches = eFeature2D.attribute("next_matches_size").as_uint();
        feature2D->nextMatches().resize(nNextMatches);

        pugi::xml_attribute  aFeature2DNextMatches = eFeature2D.child("next_matches").first_attribute();
        for (size_t j = 0; j < feature2D->nextMatches().size(); ++j)
        {
            unsigned int key;
            sscanf(aFeature2DNextMatches.name(), "next_matches_%u", &key);

            unsigned int value;
            sscanf(aFeature2DNextMatches.value(), "F2D-%u", &value);

            feature2D->nextMatches().at(key) = feature2DMap.at(value);

            aFeature2DPrevMatches = aFeature2DNextMatches.next_attribute();
        }

        pugi::xml_attribute aFeature3D = eFeature2D.attribute("feature3D");
        if (!aFeature3D.empty())
        {
            size_t feature3DIdx;
            sscanf(aFeature3D.value(), "F3D-%lu", &feature3DIdx);

            feature2D->feature3D() = feature3DMap.at(feature3DIdx);
        }

        pugi::xml_attribute aFrame = eFeature2D.attribute("frame");
        if (aFrame != 0)
        {
            size_t frameIdx;
            sscanf(aFrame.value(), "frame%lu", &frameIdx);
            feature2D->frame() = frameMap.at(frameIdx);
        }
    }

    for (size_t i = 0; i < feature3DXMLMap.size(); ++i)
    {
        Point3DFeaturePtr& feature3D = feature3DMap.at(i);
        pugi::xml_node eFeature3D = feature3DXMLMap.at(i);

        unsigned int nFeatures2D = eFeature3D.attribute("features2D_size").as_uint();
        feature3D->features2D().resize(nFeatures2D);

        pugi::xml_attribute aFeatures2DCorr = eFeature3D.child("features2D").first_attribute();
        for (size_t j = 0; j < feature3D->features2D().size(); ++j)
        {
            unsigned int key;
            sscanf(aFeatures2DCorr.name(), "features2D_%u", &key);

            unsigned int value;
            sscanf(aFeatures2DCorr.value(), "F2D-%u", &value);

            feature3D->features2D().at(key) = feature2DMap.at(value);

            aFeatures2DCorr = aFeatures2DCorr.next_attribute();
        }
    }

    return true;
}

void
SparseGraph::writeToXMLFile(const std::string& filename) const
{
    boost::filesystem::path xmlPath(filename);

    boost::filesystem::path imageDir;
    if (xmlPath.has_parent_path())
    {
        imageDir = xmlPath.parent_path();
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

    // write frame data to xml file
    pugi::xml_document doc;

    pugi::xml_node eRoot = doc.append_child("root");
    eRoot.append_attribute("cameras_size") = static_cast<unsigned int>(mFrameSegments.size());

    boost::unordered_map<Frame*,size_t> frameMap;
    boost::unordered_map<Pose*,size_t> poseMap;
    boost::unordered_map<Odometry*,size_t> odometryMap;
    boost::unordered_map<Point2DFeature*,size_t> feature2DMap;
    boost::unordered_map<Point3DFeature*,size_t> feature3DMap;

    for (size_t cameraIdx = 0; cameraIdx < mFrameSegments.size(); ++cameraIdx)
    {
        char cameraName[255];
        sprintf(cameraName, "camera%lu", cameraIdx);
        pugi::xml_node eCamera = eRoot.append_child(cameraName);

        eCamera.append_attribute("segments_size") = static_cast<unsigned int>(mFrameSegments.at(cameraIdx).size());
        for (size_t segmentIdx = 0; segmentIdx < mFrameSegments.at(cameraIdx).size(); ++segmentIdx)
        {
            const FrameSegment& segment = mFrameSegments.at(cameraIdx).at(segmentIdx);

            char segmentName[255];
            sprintf(segmentName, "segment%lu", segmentIdx);
            pugi::xml_node eSegment = eCamera.append_child(segmentName);

            eSegment.append_attribute("frames_size") = static_cast<unsigned int>(segment.size());

            pugi::xml_node eFramesSegment = eSegment.append_child("frames");

            // index all structures
            for (size_t frameIdx = 0; frameIdx < segment.size(); ++frameIdx)
            {
                const FramePtr& frame = segment.at(frameIdx);

                char key[255];
                sprintf(key, "frame%lu", frameIdx);

                char value[255];
                sprintf(value, "frame%lu", frameMap.size());

                frameMap.insert(std::make_pair(frame.get(), frameMap.size()));

                eFramesSegment.append_attribute(key) = value;

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

    pugi::xml_node eFrames = eRoot.append_child("frames");
    pugi::xml_node ePoses = eRoot.append_child("poses");
    pugi::xml_node eOdometry = eRoot.append_child("odometry");
    pugi::xml_node eFeatures2D = eRoot.append_child("features2D");
    pugi::xml_node eFeatures3D = eRoot.append_child("features3D");

    eFrames.append_attribute("size") = static_cast<unsigned int>(frameMap.size());
    ePoses.append_attribute("size") = static_cast<unsigned int>(poseMap.size());
    eOdometry.append_attribute("size") = static_cast<unsigned int>(odometryMap.size());
    eFeatures2D.append_attribute("size") = static_cast<unsigned int>(feature2DMap.size());
    eFeatures3D.append_attribute("size") = static_cast<unsigned int>(feature3DMap.size());

    // link all references
    for (boost::unordered_map<Frame*,size_t>::iterator it = frameMap.begin();
             it != frameMap.end(); ++it)
    {
        Frame* frame = it->first;

        char frameName[255];
        sprintf(frameName, "frame%lu", it->second);

        pugi::xml_node eFrame = eFrames.append_child(frameName);

        // attributes
        if (!frame->image().empty())
        {
            char imageFilename[1024];
            sprintf(imageFilename, "%s/%s.png",
                    imageDir.string().c_str(), frameName);
            cv::imwrite(imageFilename, frame->image());

            memset(imageFilename, 0, 1024);
            sprintf(imageFilename, "images/%s.png", frameName);
            eFrame.append_attribute("image") = imageFilename;
        }

        eFrame.append_attribute("id") = frame->id();

        // references
        if (frame->camera().get() != 0)
        {
            char poseName[255];
            sprintf(poseName, "pose%lu", poseMap[frame->camera().get()]);
            eFrame.append_attribute("camera") = poseName;
        }

        if (frame->odometry().get() != 0)
        {
            char odometryName[255];
            sprintf(odometryName, "odometry%lu", odometryMap[frame->odometry().get()]);
            eFrame.append_attribute("odometry") = odometryName;
        }

        if (frame->gps_ins().get() != 0)
        {
            char poseName[255];
            sprintf(poseName, "pose%lu", poseMap[frame->gps_ins().get()]);
            eFrame.append_attribute("gps_ins") = poseName;
        }

        eFrame.append_attribute("features2D_size") = static_cast<unsigned int>(frame->features2D().size());

        pugi::xml_node eFeatures2DFrame = eFrame.append_child("features2D");

        const std::vector<Point2DFeaturePtr>& features2D = frame->features2D();
        for (size_t i = 0; i < features2D.size(); ++i)
        {
            const Point2DFeaturePtr& feature2D = features2D.at(i);

            boost::unordered_map<Point2DFeature*,size_t>::iterator itF2D = feature2DMap.find(feature2D.get());
            if (itF2D == feature2DMap.end())
            {
//                std::cout << "# WARNING: Frame::features2D: Point2DFeature instance was not found in map.\n";
                continue;
            }

            char feature2DName[255];
            sprintf(feature2DName, "F2D-%lu", itF2D->second);

            char keyName[255];
            sprintf(keyName, "features2D_%lu", i);
            eFeatures2DFrame.append_attribute(keyName) = feature2DName;
        }

        eFrame.append_attribute("features3D_size") = static_cast<unsigned int>(frame->features3D().size());

        pugi::xml_node eFeatures3DFrame = eFrame.append_child("features3D");

        const std::vector<Point3DFeaturePtr>& features3D = frame->features3D();
        for (size_t i = 0; i < features3D.size(); ++i)
        {
            const Point3DFeaturePtr& feature3D = features3D.at(i);

            boost::unordered_map<Point3DFeature*,size_t>::iterator itF3D = feature3DMap.find(feature3D.get());
            if (itF3D == feature3DMap.end())
            {
//                std::cout << "# WARNING: Frame::features3D: Point3DFeature instance was not found in map.\n";
                continue;
            }

            char feature3DName[255];
            sprintf(feature3DName, "F3D-%lu", itF3D->second);

            char keyName[255];
            sprintf(keyName, "features3D_%lu", i);
            eFeatures3DFrame.append_attribute(keyName) = feature3DName;
        }
    }

    for (boost::unordered_map<Pose*, size_t>::iterator it = poseMap.begin();
            it != poseMap.end(); ++it)
    {
        Pose* pose = it->first;

        char poseName[255];
        sprintf(poseName, "pose%lu", it->second);
        pugi::xml_node ePose = ePoses.append_child(poseName);

        std::ostringstream oss;
        oss << pose->timeStamp();
        ePose.append_attribute("timestamp") = oss.str().c_str();

        const double* const q = pose->rotationData();
        ePose.append_attribute("q_x") = q[0];
        ePose.append_attribute("q_y") = q[1];
        ePose.append_attribute("q_z") = q[2];
        ePose.append_attribute("q_w") = q[3];

        oss << std::fixed << std::setprecision(10);

        const double* const t = pose->translationData();
        oss.str(""); oss.clear(); oss << t[0];
        ePose.append_attribute("t_x") = oss.str().c_str();
        oss.str(""); oss.clear(); oss << t[1];
        ePose.append_attribute("t_y") = oss.str().c_str();
        oss.str(""); oss.clear(); oss << t[2];
        ePose.append_attribute("t_z") = oss.str().c_str();
    }

    for (boost::unordered_map<Odometry*, size_t>::iterator it = odometryMap.begin();
            it != odometryMap.end(); ++it)
    {
        Odometry* odometry = it->first;

        char odometryName[255];
        sprintf(odometryName, "odometry%lu", it->second);
        pugi::xml_node eOdometry = eOdometry.append_child(odometryName);

        std::ostringstream oss;
        oss << odometry->timeStamp();
        eOdometry.append_attribute("timestamp") = oss.str().c_str();

        eOdometry.append_attribute("x") = odometry->x();
        eOdometry.append_attribute("y") = odometry->y();
        eOdometry.append_attribute("z") = odometry->z();
        eOdometry.append_attribute("yaw") = odometry->yaw();
        eOdometry.append_attribute("pitch") = odometry->pitch();
        eOdometry.append_attribute("roll") = odometry->roll();
    }

    for (boost::unordered_map<Point2DFeature*,size_t>::iterator it = feature2DMap.begin();
             it != feature2DMap.end(); ++it)
    {
        Point2DFeature* feature2D = it->first;

        char feature2DName[255];
        sprintf(feature2DName, "F2D-%lu", it->second);
        pugi::xml_node eFeature2D = eFeatures2D.append_child(feature2DName);

        // attributes
        pugi::xml_node eDtor = eFeature2D.append_child("dtor");

        const cv::Mat& dtor = feature2D->descriptor();

        eDtor.append_attribute("type") = dtor.type();
        eDtor.append_attribute("rows") = dtor.rows;
        eDtor.append_attribute("cols") = dtor.cols;

        pugi::xml_node eMat = eDtor.append_child("mat");

        for (int r = 0; r < dtor.rows; ++r)
        {
            for (int c = 0; c < dtor.cols; ++c)
            {
                char elementName[255];
                sprintf(elementName, "m-%d-%d", r, c);

                switch (dtor.type())
                {
                case CV_8U:
                    eMat.append_attribute(elementName) = static_cast<int>(dtor.at<unsigned char>(r,c));
                    break;
                case CV_8S:
                    eMat.append_attribute(elementName) = static_cast<int>(dtor.at<char>(r,c));
                    break;
                case CV_16U:
                    eMat.append_attribute(elementName) = static_cast<int>(dtor.at<unsigned short>(r,c));
                    break;
                case CV_16S:
                    eMat.append_attribute(elementName) = static_cast<int>(dtor.at<short>(r,c));
                    break;
                case CV_32S:
                    eMat.append_attribute(elementName) = dtor.at<int>(r,c);
                    break;
                case CV_32F:
                    eMat.append_attribute(elementName) = static_cast<double>(dtor.at<float>(r,c));
                    break;
                case CV_64F:
                default:
                    eMat.append_attribute(elementName) = dtor.at<double>(r,c);
                }
            }
        }

        eFeature2D.append_attribute("kp_angle") = feature2D->keypoint().angle;
        eFeature2D.append_attribute("kp_class_id") = feature2D->keypoint().class_id;
        eFeature2D.append_attribute("kp_octave") = feature2D->keypoint().octave;
        eFeature2D.append_attribute("kp_x") = feature2D->keypoint().pt.x;
        eFeature2D.append_attribute("kp_y") = feature2D->keypoint().pt.y;
        eFeature2D.append_attribute("kp_response") = feature2D->keypoint().response;
        eFeature2D.append_attribute("kp_size") = feature2D->keypoint().size;
        eFeature2D.append_attribute("index") = feature2D->index();
        eFeature2D.append_attribute("best_prev_match_idx") = feature2D->bestPrevMatchIdx();
        eFeature2D.append_attribute("best_next_match_idx") = feature2D->bestNextMatchIdx();

        // references
        pugi::xml_node eFeature2DPrevMatches = eFeature2D.append_child("prev_matches");

        unsigned int nValidPrevMatches = 0;
        for (size_t i = 0; i < feature2D->prevMatches().size(); ++i)
        {
            char keyName[255];
            sprintf(keyName, "prev_matches_%lu", i);

            boost::unordered_map<Point2DFeature*,size_t>::iterator itF2D = feature2DMap.find(feature2D->prevMatches().at(i).get());
            if (itF2D == feature2DMap.end())
            {
//                std::cout << "# WARNING: Feature2D::prevMatches: Point2DFeature instance was not found in map.\n";
                continue;
            }

            char feature2DName[255];
            sprintf(feature2DName, "F2D-%lu", itF2D->second);
            eFeature2DPrevMatches.append_attribute(keyName) = feature2DName;

            ++nValidPrevMatches;
        }
        eFeature2D.append_attribute("prev_matches_size") = nValidPrevMatches;

        pugi::xml_node eFeature2DNextMatches = eFeature2D.append_child("next_matches");

        unsigned int nValidNextMatches = 0;
        for (size_t i = 0; i < feature2D->nextMatches().size(); ++i)
        {
            char keyName[255];
            sprintf(keyName, "next_matches_%lu", i);

            boost::unordered_map<Point2DFeature*,size_t>::iterator itF2D = feature2DMap.find(feature2D->nextMatches().at(i).get());
            if (itF2D == feature2DMap.end())
            {
//                std::cout << "# WARNING: Feature2D::nextMatches: Point2DFeature instance was not found in map.\n";
                continue;
            }

            char feature2DName[255];
            sprintf(feature2DName, "F2D-%lu", itF2D->second);
            eFeature2DNextMatches.append_attribute(keyName) = feature2DName;

            ++nValidNextMatches;
        }
        eFeature2D.append_attribute("next_matches_size") = nValidNextMatches;

        if (feature2D->feature3D().get() != 0)
        {
            boost::unordered_map<Point3DFeature*,size_t>::iterator itF3D = feature3DMap.find(feature2D->feature3D().get());
            if (itF3D == feature3DMap.end())
            {
//                std::cout << "# WARNING: Feature2D::feature3D: Point3DFeature instance was not found in map.\n";
                continue;
            }

            char feature3DName[255];
            sprintf(feature3DName, "F3D-%lu", itF3D->second);
            eFeature2D.append_attribute("feature3D") = feature3DName;
        }

        if (feature2D->frame().get() != 0)
        {
            char frameName[255];
            sprintf(frameName, "frame%lu", frameMap[feature2D->frame().get()]);
            eFeature2D.append_attribute("frame") = frameName;
        }
    }

    for (boost::unordered_map<Point3DFeature*,size_t>::iterator it = feature3DMap.begin();
             it != feature3DMap.end(); ++it)
    {
        Point3DFeature* feature3D = it->first;

        // attributes
        char feature3DName[255];
        sprintf(feature3DName, "F3D-%lu", it->second);
        pugi::xml_node eFeature3D = eFeatures3D.append_child(feature3DName);

        const Eigen::Vector3d& P = feature3D->point();
        eFeature3D.append_attribute("x") = P(0);
        eFeature3D.append_attribute("y") = P(1);
        eFeature3D.append_attribute("z") = P(2);

        // references
        pugi::xml_node eFeature2DCorr = eFeature3D.append_child("features2D");

        unsigned int nValidFeatures2D = 0;
        for (size_t i = 0; i < feature3D->features2D().size(); ++i)
        {
            char keyName[255];
            sprintf(keyName, "features2D_%lu", i);

            boost::unordered_map<Point2DFeature*,size_t>::iterator itF2D = feature2DMap.find(feature3D->features2D().at(i).get());
            if (itF2D == feature2DMap.end())
            {
//                std::cout << "# WARNING: Feature3D::features2D: Point2DFeature instance was not found in map.\n";
                continue;
            }

            char feature2DName[255];
            sprintf(feature2DName, "F2D-%lu", itF2D->second);
            eFeature2DCorr.append_attribute(keyName) = feature2DName;

            ++nValidFeatures2D;
        }
        eFeature3D.append_attribute("features2D_size") = nValidFeatures2D;
    }

    doc.save_file(filename.c_str());
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

void
SparseGraph::XMLToFrames(pugi::xml_node& parent, unsigned int count,
                         FrameSegment& map,
                         std::vector<pugi::xml_node>& xmlMap,
                         const boost::filesystem::path& rootDir) const
{
    map.resize(count);
    xmlMap.resize(count);

    pugi::xml_node eFrame = eFrame = parent.first_child();

    for (unsigned int i = 0; i < count; ++i)
    {
        unsigned int frameIdx;
        sscanf(eFrame.name(), "frame%u", &frameIdx);

        FramePtr frame(new Frame);

        pugi::xml_attribute aImageFilename = eFrame.attribute("image");
        if (!aImageFilename.empty())
        {
            boost::filesystem::path imagePath = rootDir;
            imagePath /= aImageFilename.value();

            frame->image() = cv::imread(imagePath.string().c_str(), -1);
        }

        frame->id() = eFrame.attribute("id").as_uint();

        map.at(frameIdx) = frame;
        xmlMap.at(frameIdx) = eFrame;

        eFrame = eFrame.next_sibling();
    }
}

void
SparseGraph::XMLToPoint2DFeatures(pugi::xml_node& parent, unsigned int count,
                                  std::vector<Point2DFeaturePtr>& map,
                                  std::vector<pugi::xml_node>& xmlMap) const
{
    map.resize(count);
    xmlMap.resize(count);

    pugi::xml_node eFeature2D = parent.first_child();

    for (unsigned int i = 0; i < count; ++i)
    {
        unsigned int featureIdx;
        sscanf(eFeature2D.name(), "F2D-%u", &featureIdx);

        Point2DFeaturePtr feature2D(new Point2DFeature);

        pugi::xml_node eDtor = eFeature2D.child("dtor");

        int type = eDtor.attribute("type").as_int();
        int rows = eDtor.attribute("rows").as_int();
        int cols = eDtor.attribute("cols").as_int();

        feature2D->descriptor() = cv::Mat(rows, cols, type);

        cv::Mat& dtor = feature2D->descriptor();

        pugi::xml_attribute aMat = eDtor.child("mat").first_attribute();
        int matSize = rows * cols;
        for (int j = 0; j < matSize; ++j)
        {
            int r, c;
            sscanf(aMat.name(), "m-%d-%d", &r, &c);

            int iElement;
            double dElement;

            switch (dtor.type())
            {
            case CV_8U:
            case CV_8S:
            case CV_16U:
            case CV_16S:
            case CV_32S:
                iElement = aMat.as_int();
                break;
            case CV_32F:
            case CV_64F:
            default:
                dElement = aMat.as_double();
            }

            switch (dtor.type())
            {
            case CV_8U:
                dtor.at<unsigned char>(r,c) = static_cast<unsigned char>(iElement);
                break;
            case CV_8S:
                dtor.at<char>(r,c) = static_cast<char>(iElement);
                break;
            case CV_16U:
                dtor.at<unsigned short>(r,c) = static_cast<unsigned short>(iElement);
                break;
            case CV_16S:
                dtor.at<short>(r,c) = static_cast<short>(iElement);
                break;
            case CV_32S:
                dtor.at<int>(r,c) = iElement;
                break;
            case CV_32F:
                dtor.at<float>(r,c) = static_cast<float>(dElement);
                break;
            case CV_64F:
            default:
                dtor.at<double>(r,c) = dElement;
            }

            aMat = aMat.next_attribute();
        }

        feature2D->keypoint().angle = eFeature2D.attribute("kp_angle").as_float();
        feature2D->keypoint().class_id = eFeature2D.attribute("kp_class_id").as_int();
        feature2D->keypoint().octave = eFeature2D.attribute("kp_octave").as_int();
        feature2D->keypoint().pt.x = eFeature2D.attribute("kp_x").as_float();
        feature2D->keypoint().pt.y = eFeature2D.attribute("kp_y").as_float();
        feature2D->keypoint().response = eFeature2D.attribute("kp_response").as_float();
        feature2D->keypoint().size = eFeature2D.attribute("kp_size").as_float();
        feature2D->index() = eFeature2D.attribute("index").as_uint();
        feature2D->bestPrevMatchIdx() = eFeature2D.attribute("best_prev_match_idx").as_int();
        feature2D->bestNextMatchIdx() = eFeature2D.attribute("best_next_match_idx").as_int();

        map.at(featureIdx) = feature2D;
        xmlMap.at(featureIdx) = eFeature2D;

        eFeature2D = eFeature2D.next_sibling();
    }
}

void
SparseGraph::XMLToPoint3DFeatures(pugi::xml_node& parent, unsigned int count,
                                  std::vector<Point3DFeaturePtr>& map,
                                  std::vector<pugi::xml_node>& xmlMap) const
{
    map.resize(count);
    xmlMap.resize(count);

    pugi::xml_node eFeature3D = parent.first_child();

    for (unsigned int i = 0; i < count; ++i)
    {
        unsigned int featureIdx;
        sscanf(eFeature3D.name(), "F3D-%u", &featureIdx);

        Point3DFeaturePtr feature3D(new Point3DFeature);

        Eigen::Vector3d& P = feature3D->point();
        P(0) = eFeature3D.attribute("x").as_double();
        P(1) = eFeature3D.attribute("y").as_double();
        P(2) = eFeature3D.attribute("z").as_double();

        map.at(featureIdx) = feature3D;
        xmlMap.at(featureIdx) = eFeature3D;

        eFeature3D = eFeature3D.next_sibling();
    }
}

void
SparseGraph::XMLToPoses(pugi::xml_node& parent, unsigned int count,
                        std::vector<PosePtr>& map) const
{
    map.resize(count);

    pugi::xml_node ePose = parent.first_child();

    for (unsigned int i = 0; i < count; ++i)
    {
        unsigned int poseIdx;
        sscanf(ePose.name(), "pose%u", &poseIdx);

        PosePtr pose(new Pose);

        std::istringstream iss(ePose.attribute("timestamp").value());
        iss >> pose->timeStamp();

        double q[4];
        q[0] = ePose.attribute("q_x").as_double();
        q[1] = ePose.attribute("q_y").as_double();
        q[2] = ePose.attribute("q_z").as_double();
        q[3] = ePose.attribute("q_w").as_double();

        memcpy(pose->rotationData(), q, sizeof(double) * 4);

        double t[3];
        iss.str(ePose.attribute("t_x").value()); iss.clear();
        iss >> t[0];
        iss.str(ePose.attribute("t_y").value()); iss.clear();
        iss >> t[1];
        iss.str(ePose.attribute("t_z").value()); iss.clear();
        iss >> t[2];

        memcpy(pose->translationData(), t, sizeof(double) * 3);

        map.at(poseIdx) = pose;

        ePose = ePose.next_sibling();
    }
}

void
SparseGraph::XMLToOdometry(pugi::xml_node& parent, unsigned int count,
                           std::vector<OdometryPtr>& map) const
{
    map.resize(count);

    pugi::xml_node eOdometry = parent.first_child();

    for (unsigned int i = 0; i < count; ++i)
    {
        unsigned int odometryIdx;
        sscanf(eOdometry.name(), "odometry%u", &odometryIdx);

        OdometryPtr odometry(new Odometry);

        std::istringstream iss(eOdometry.attribute("timestamp").value());
        iss >> odometry->timeStamp();

        odometry->x() = eOdometry.attribute("x").as_double();
        odometry->y() = eOdometry.attribute("y").as_double();
        odometry->z() = eOdometry.attribute("z").as_double();
        odometry->yaw() = eOdometry.attribute("yaw").as_double();
        odometry->pitch() = eOdometry.attribute("pitch").as_double();
        odometry->roll() = eOdometry.attribute("roll").as_double();

        map.at(odometryIdx) = odometry;

        eOdometry = eOdometry.next_sibling();
    }
}

}
