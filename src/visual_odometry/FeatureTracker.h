#ifndef FEATURETRACKER_H
#define FEATURETRACKER_H

#include <Eigen/Dense>
#include <opencv2/features2d/features2d.hpp>

#include "camodocal/camera_models/Camera.h"
#include "camodocal/sparse_graph/SparseGraph.h"
#include "../features2d/ORBGPU.h"
#include "../features2d/SurfGPU.h"
#include "SlidingWindowBA.h"

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

    int m_cameraIdx;
    cv::Mat m_cameraMatrix;

    cv::Mat m_sketch;

    float m_maxDistanceRatio;

    cv::Ptr<cv::FeatureDetector> m_featureDetector;
    cv::Ptr<cv::DescriptorExtractor> m_descriptorExtractor;
    cv::Ptr<cv::DescriptorMatcher> m_descriptorMatcher;

    cv::Ptr<SurfGPU> m_SURF_GPU;
    cv::Ptr<ORBGPU> m_ORB_GPU;

    DetectorType m_detectorType;
    DescriptorType m_descriptorType;
    MatchTestType m_matchTestType;
    bool m_preprocess;

    bool m_verbose;
};

class TemporalFeatureTracker: public FeatureTracker
{
public:
    TemporalFeatureTracker(const CameraConstPtr& camera,
                           DetectorType detectorType = ORB_DETECTOR,
                           DescriptorType descriptorType = ORB_DESCRIPTOR,
                           MatchTestType matchTestType = RATIO,
                           bool preprocess = false,
                           const Eigen::Matrix4d& globalCameraPose = Eigen::Matrix4d::Identity());
    bool addFrame(FramePtr& frame, const cv::Mat& mask);
    void clear(void);

    void runBundleAdjustment(void);

    void getMatches(std::vector<cv::Point2f>& matchedPoints,
                    std::vector<cv::Point2f>& matchedPointsPrev) const;
    std::vector<FramePtr>& getFrames(void);
    const std::vector<FramePtr>& getFrames(void) const;
    const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> >& getPoses(void) const;

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > getScenePoints(void) const;

protected:
    void visualizeTracks(void);

    const CameraConstPtr k_camera;

    cv::Mat m_image;
    cv::Mat m_mask;

    bool m_init;

    std::vector<cv::KeyPoint> m_kpts, m_kptsPrev;
    cv::Mat m_dtor, m_dtorPrev;
    FramePtr m_framePrev;

    std::vector<Point2DFeaturePtr> m_pointFeatures;

    SlidingWindowBA m_BA;
    std::vector<FramePtr> m_frames;
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > m_poses;

    cv::Mat m_matchingMask;
    const float k_maxDelta;
    const int k_minFeatureCorrespondences;
    const double k_nominalFocalLength;
    const double k_reprojErrorThresh;
};

}

#endif
