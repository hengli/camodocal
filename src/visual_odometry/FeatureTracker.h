#ifndef FEATURETRACKER_H
#define FEATURETRACKER_H

#include <Eigen/Dense>
#include <opencv2/features2d/features2d.hpp>

#include "camodocal/camera_models/Camera.h"
#include "camodocal/sparse_graph/SparseGraph.h"
#include "SlidingWindowBA.h"
#include "ORBGPU.h"
#include "SurfGPU.h"

namespace camodocal
{

enum DetectorType
{
    FAST_DETECTOR = 0x0,
    ORB_DETECTOR = 0x1,
    ORB_GPU_DETECTOR = 0x11,
    STAR_DETECTOR = 0x2,
    SURF_DETECTOR = 0x3,
    SURF_GPU_DETECTOR = 0x13
};

enum DescriptorType
{
    BRISK_DESCRIPTOR = 0x0,
    ORB_DESCRIPTOR = 0x1,
    ORB_GPU_DESCRIPTOR = 0x11,
    SURF_DESCRIPTOR = 0x2,
    SURF_GPU_DESCRIPTOR = 0x12
};

enum MatchTestType
{
    BEST_MATCH = 0x0,       // best match
    BEST_MATCH_GPU = 0x10,  // best match
    RADIUS = 0x1,           // radius
    RADIUS_GPU = 0x11,      // radius
    RATIO = 0x2,            // ratio test used by Lowe in SIFT paper
    RATIO_GPU = 0x12        // ratio test used by Lowe in SIFT paper
};

class FeatureTracker
{
public:
    FeatureTracker(DetectorType detectorType = ORB_DETECTOR,
                   DescriptorType descriptorType = ORB_DESCRIPTOR,
                   MatchTestType matchTestType = RATIO,
                   bool preprocess = false);

    cv::Mat& cameraMatrix(void);

    const cv::Mat& getSketch(void) const;

    void setMaxDistanceRatio(float maxDistanceRatio);
    void setVerbose(bool verbose);

protected:
    void preprocessImage(cv::Mat& image, const cv::Mat& mask = cv::Mat()) const;
    void detectFeatures(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints,
                        const cv::Mat& mask = cv::Mat());
    void computeDescriptors(const cv::Mat& image,
                            std::vector<cv::KeyPoint>& keypoints,
                            cv::Mat& descriptors);
    void matchPointFeaturesWithBestMatchTest(const cv::Mat& dtor1,
                                             const cv::Mat& dtor2,
                                             std::vector<std::vector<cv::DMatch> >& matches,
                                             const cv::Mat& mask = cv::Mat());
    void matchPointFeaturesWithRadiusTest(const cv::Mat& dtor1,
                                          const cv::Mat& dtor2,
                                          std::vector<std::vector<cv::DMatch> >& matches,
                                          const cv::Mat& mask = cv::Mat(),
                                          float maxDistance = 0.01f);
    void matchPointFeaturesWithRatioTest(const cv::Mat& dtor1,
                                         const cv::Mat& dtor2,
                                         std::vector<std::vector<cv::DMatch> >& matches,
                                         const cv::Mat& mask = cv::Mat());

    void windowedMatchingMask(const std::vector<cv::KeyPoint>& keypoints1,
                              const std::vector<cv::KeyPoint>& keypoints2,
                              float maxDeltaX, float maxDeltaY,
                              cv::Mat& mask) const;

    int mCameraIdx;
    cv::Mat mCameraMatrix;

    cv::Mat mSketch;

    float mMaxDistanceRatio;

    cv::Ptr<cv::FeatureDetector> mFeatureDetector;
    cv::Ptr<cv::DescriptorExtractor> mDescriptorExtractor;
    cv::Ptr<cv::DescriptorMatcher> mDescriptorMatcher;

    cv::Ptr<SurfGPU> mSURF_GPU;
    cv::Ptr<ORBGPU> mORB_GPU;

    DetectorType mDetectorType;
    DescriptorType mDescriptorType;
    MatchTestType mMatchTestType;
    bool mPreprocess;

    bool mVerbose;
};

class TemporalFeatureTracker: public FeatureTracker
{
public:
    TemporalFeatureTracker(const CameraConstPtr& camera,
                           DetectorType detectorType = ORB_DETECTOR,
                           DescriptorType descriptorType = ORB_DESCRIPTOR,
                           MatchTestType matchTestType = RATIO,
                           bool preprocess = false);
    bool addFrame(FramePtr& frame, const cv::Mat& mask,
                  Eigen::Matrix3d& R_rel, Eigen::Vector3d& t_rel);
    void clear(void);

    void getMatches(std::vector<cv::Point2f>& matchedPoints,
                    std::vector<cv::Point2f>& matchedPointsPrev) const;
    std::vector<FramePtr>& getFrames(void);
    const std::vector<FramePtr>& getFrames(void) const;
    const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> >& getPoses(void) const;

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > getScenePoints(void) const;

protected:
    bool computeVO(Eigen::Matrix3d& R_rel, Eigen::Vector3d& t_rel, cv::Mat& inliers);

    int findInliers(const Eigen::Matrix3d& R_rel, const Eigen::Vector3d& t_rel,
                    double reprojErrorThresh = 1.0);

    void rectifyImagePoint(const cv::Point2f& src, cv::Point2f& dst) const;

    void visualizeTracks(void);

    const CameraConstPtr kCamera;

    cv::Mat mImage;
    cv::Mat mMask;

    bool mInit;

    std::vector<cv::KeyPoint> mKpts, mKptsPrev;
    cv::Mat mDtor, mDtorPrev;

    std::vector<Point2DFeaturePtr> mPointFeatures;

    SlidingWindowBA m_BA;
    std::vector<FramePtr> mFrames;
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > mPoses;

    cv::Mat mMatchingMask;
    const float kMaxDelta;
    const int kMinFeatureCorrespondences;
    const double kNominalFocalLength;
    const double kReprojErrorThresh;
};

class CameraRigTemporalFeatureTracker: public FeatureTracker
{
public:
    CameraRigTemporalFeatureTracker(const std::vector<CameraConstPtr>& cameras,
                                    DetectorType detectorType = ORB_DETECTOR,
                                    DescriptorType descriptorType = ORB_DESCRIPTOR,
                                    MatchTestType matchTestType = RATIO,
                                    bool preprocess = false);
    bool addFrame(const std::vector<cv::Mat>& images,
                  const std::vector<cv::Mat>& masks);
    void clear(void);

    const cv::Mat& getSketch(int idx) const;

protected:
    class CameraMetadata
    {
    public:
        CameraMetadata(const CameraConstPtr& _camera)
         : camera(_camera)
        {

        }

        CameraConstPtr camera;
        cv::Mat image;
        cv::Mat mask;
        std::vector<cv::KeyPoint> kpts, kptsPrev;
        cv::Mat dtor, dtorPrev;
        std::vector<Point2DFeaturePtr> pointFeatures;
        cv::Mat sketch;
    };

    void processImage(const cv::Mat& image, const cv::Mat& mask,
                      CameraMetadata* metadata);

    void rectifyImagePoint(const CameraConstPtr& camera,
                           const cv::Point2f& src, cv::Point2f& dst) const;

    void visualizeTracks(void);

    std::vector<CameraMetadata> mCameraMetadata;

    cv::Mat mMatchingMask;
    const float kMaxDelta;
    const int kMinFeatureCorrespondences;
    const double kNominalFocalLength;
    const double kReprojErrorThresh;
};

class StereoFeatureTracker: public FeatureTracker
{
public:
    StereoFeatureTracker(DetectorType detectorType = ORB_DETECTOR,
                         DescriptorType descriptorType = ORB_DESCRIPTOR,
                         MatchTestType matchTestType = RATIO, 
                         bool preprocess = false);
    void addStereoFrame(const cv::Mat& imageLeft,
                        const cv::Mat& imageRight,
                        const cv::Mat& maskLeft = cv::Mat(),
                        const cv::Mat& maskRight = cv::Mat());

    const std::vector<Point2DFeatureLeftPtr>& getPointFeaturesLeft(void) const;
    const std::vector<Point2DFeatureRightPtr>& getPointFeaturesRight(void) const;

    virtual void getMatches(std::vector<cv::Point2f>& leftMatchedPoints,
                            std::vector<cv::Point2f>& rightMatchedPoints) const;
protected: 
    void copyImagesToSketch(const cv::Mat& imageLeft, const cv::Mat& imageRight);
    void visualizeTracks(void);

    cv::Mat mImageLeft, mImageRight;
    cv::Mat mMaskLeft, mMaskRight;

    std::vector<cv::KeyPoint> mKptsLeft, mKptsRight;
    cv::Mat mDtorLeft, mDtorRight;

    std::vector<Point2DFeatureLeftPtr> mPointFeaturesLeft;
    std::vector<Point2DFeatureRightPtr> mPointFeaturesRight;

    cv::Mat mMatchingMask;
    const float kMaxDelta;
}; 

}

#endif
