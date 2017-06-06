#include <boost/make_shared.hpp>
#include <boost/thread.hpp>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

#ifdef HAVE_CUDA
#ifdef HAVE_OPENCV3

    //////////////////
    // CUDA + OPENCV3
    //////////////////
#include <opencv2/cudafeatures2d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#else  // HAVE_OPENCV3

    //////////////////
    // CUDA + OPENCV2
    //////////////////
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/gpu.hpp>
#include <opencv2/gpu/gpu.hpp>

#endif // HAVE_OPENCV3
#else  // HAVE_CUDA

#ifdef HAVE_OPENCV3

    //////////////////
    // OPENCV3
    //////////////////
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#else // HAVE_OPENCV3

    //////////////////
    // OPENCV2
    //////////////////
#include <opencv2/nonfree/features2d.hpp>

#endif // HAVE_OPENCV3
#endif // HAVE_CUDA

#ifndef HAVE_OPENCV3
#include <opencv2/legacy/legacy.hpp>
#endif // HAVE_OPENCV3


#include "../brisk/include/brisk/brisk.h"
#include "../camera_models/CostFunctionFactory.h"
#include "../gpl/gpl.h"
#include "camodocal/EigenUtils.h"
#include "../gpl/OpenCVUtils.h"
#include "../npoint/five-point/five-point.hpp"
#include "ceres/ceres.h"
#include "FeatureTracker.h"

namespace camodocal
{

/// @todo when DETECTOR and DESCRIPTOR match, should only one be created and given to both cv::Ptr objects?
FeatureTracker::FeatureTracker(DetectorType detectorType,
                               DescriptorType descriptorType,
                               MatchTestType matchTestType,
                               bool preprocess)
 : m_maxDistanceRatio(0.7f)
 , m_detectorType(detectorType)
 , m_descriptorType(descriptorType)
 , m_matchTestType(matchTestType)
 , m_preprocess(preprocess)
 , m_verbose(false)
{
    bool crossCheck = false;
    static const int orbNFeatures = 1000;
    static const int surfNFeatures = 500;
    static const int surfGPU_NFeatures = 200;

    // FEATURE DETECTORS
#ifdef HAVE_OPENCV3
    switch (detectorType)
    {
    case FAST_DETECTOR:
        m_featureDetector = cv::FastFeatureDetector::create(25);
        break;
    case ORB_DETECTOR:
        m_featureDetector = cv::ORB::create(orbNFeatures);
        break;
    case ORB_GPU_DETECTOR:
        m_ORB_GPU = ORBGPU::instance(orbNFeatures);
        break;
    case STAR_DETECTOR:
//        m_featureDetector = new cv::GridAdaptedFeatureDetector(new cv::StarDetector(15, 5, 10, 8, 20), 500, 3, 2);
        m_featureDetector = cv::xfeatures2d::StarDetector::create(16, 25, 10, 8, 5);
        break;
    case SURF_GPU_DETECTOR:
        m_SURF_GPU = SurfGPU::instance(surfGPU_NFeatures);
        break;
    case SURF_DETECTOR:
    default:
        m_featureDetector = cv::xfeatures2d::SurfFeatureDetector::create(surfNFeatures, 5, 2);
    }
#else // HAVE_OPENCV3
    switch (detectorType)
    {
    case FAST_DETECTOR:
        m_featureDetector = cv::Ptr<cv::FeatureDetector>(new cv::FastFeatureDetector(25));
        break;
    case ORB_DETECTOR:
        m_featureDetector = cv::Ptr<cv::FeatureDetector>(new cv::OrbFeatureDetector(orbNFeatures));
        break;
    case ORB_GPU_DETECTOR:
        m_ORB_GPU = ORBGPU::instance(orbNFeatures);
        break;
    case STAR_DETECTOR:
//        m_featureDetector = new cv::GridAdaptedFeatureDetector(new cv::StarDetector(15, 5, 10, 8, 20), 500, 3, 2);
        m_featureDetector = cv::Ptr<cv::FeatureDetector>(new cv::StarDetector(16, 25, 10, 8, 5));
        break;
    case SURF_GPU_DETECTOR:
        m_SURF_GPU = SurfGPU::instance(surfGPU_NFeatures);
        break;
    case SURF_DETECTOR:
    default:
        m_featureDetector = cv::Ptr<cv::FeatureDetector>(new cv::SurfFeatureDetector(surfNFeatures, 5, 2));
    }

#endif // HAVE_OPENCV3





    // FEATURE DESCRIPTORS
#ifdef HAVE_OPENCV3
    switch (descriptorType)
    {
    case BRISK_DESCRIPTOR:
        m_descriptorExtractor = cv::Ptr<cv::DescriptorExtractor>(new cv::BriskDescriptorExtractor);
        m_descriptorMatcher = cv::Ptr<cv::DescriptorMatcher>(new cv::BFMatcher(cv::NORM_HAMMING, crossCheck));
        break;
    case ORB_DESCRIPTOR:
        m_descriptorExtractor =  cv::ORB::create(orbNFeatures);
        m_descriptorMatcher = cv::Ptr<cv::DescriptorMatcher>(new cv::BFMatcher(cv::NORM_HAMMING, crossCheck));
        break;
    case ORB_GPU_DESCRIPTOR:
        m_ORB_GPU = ORBGPU::instance(orbNFeatures);
        break;
    case SURF_GPU_DESCRIPTOR:
        m_SURF_GPU = SurfGPU::instance(surfGPU_NFeatures);
        break;
    case SURF_DESCRIPTOR:
    default:
        m_descriptorExtractor = cv::xfeatures2d::SurfFeatureDetector::create(surfNFeatures, 5, 2);
        m_descriptorMatcher = cv::Ptr<cv::DescriptorMatcher>(new cv::BFMatcher(cv::NORM_L2, crossCheck));
    }
#else // HAVE_OPENCV3
    switch (descriptorType)
    {
    case BRISK_DESCRIPTOR:
        m_descriptorExtractor = cv::Ptr<cv::DescriptorExtractor>(new cv::BriskDescriptorExtractor);
        m_descriptorMatcher = cv::Ptr<cv::DescriptorMatcher>(new cv::BFMatcher(cv::NORM_HAMMING, crossCheck));
        break;
    case ORB_DESCRIPTOR:
        m_descriptorExtractor = cv::Ptr<cv::DescriptorExtractor>(new cv::OrbDescriptorExtractor);
        m_descriptorMatcher = cv::Ptr<cv::DescriptorMatcher>(new cv::BFMatcher(cv::NORM_HAMMING, crossCheck));
        break;
    case ORB_GPU_DESCRIPTOR:
        m_ORB_GPU = ORBGPU::instance(orbNFeatures);
        break;
    case SURF_GPU_DESCRIPTOR:
        m_SURF_GPU = SurfGPU::instance(surfGPU_NFeatures);
        break;
    case SURF_DESCRIPTOR:
    default:
        m_descriptorExtractor = cv::Ptr<cv::DescriptorExtractor>(new cv::SurfDescriptorExtractor(5, 2));
        m_descriptorMatcher = cv::Ptr<cv::DescriptorMatcher>(new cv::BFMatcher(cv::NORM_L2, crossCheck));
    }

#endif // HAVE_OPENCV3
}

cv::Mat&
FeatureTracker::cameraMatrix(void)
{
    return m_cameraMatrix;
}

const cv::Mat&
FeatureTracker::getSketch(void) const
{
    return m_sketch;
}

void
FeatureTracker::setMaxDistanceRatio(float maxDistanceRatio)
{
    m_maxDistanceRatio = maxDistanceRatio;
}

void
FeatureTracker::setVerbose(bool verbose)
{
    m_verbose = verbose;
}

void
FeatureTracker::preprocessImage(cv::Mat& image, const cv::Mat& mask) const
{
    if (!image.empty())
    {
        equalizeHist(image, image, mask);
    }
}

void
FeatureTracker::detectFeatures(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints,
                               const cv::Mat& mask)
{
    double ts = timeInSeconds();

    switch (m_detectorType)
    {
    case ORB_GPU_DETECTOR:
    {
        m_ORB_GPU->detect(image, keypoints, mask);
        break;
    }
    case SURF_GPU_DETECTOR:
    {
        m_SURF_GPU->detect(image, keypoints, mask);
        break;
    }
    default:
        m_featureDetector->detect(image, keypoints, mask);
    }

    if (m_verbose)
    {
        std::cout << "# INFO: Feature detection took " << timeInSeconds() - ts << "s." << std::endl;
    }
}

void
FeatureTracker::computeDescriptors(const cv::Mat& image,
                                   std::vector<cv::KeyPoint>& keypoints,
                                   cv::Mat& descriptors)
{
    double ts = timeInSeconds();

    switch (m_descriptorType)
    {
    case ORB_GPU_DESCRIPTOR:
    {
        m_ORB_GPU->compute(image, keypoints, descriptors);
        break;
    }
    case SURF_GPU_DESCRIPTOR:
    {
        m_SURF_GPU->compute(image, keypoints, descriptors);
        break;
    }
    default:
        m_descriptorExtractor->compute(image, keypoints, descriptors);
    }

    if (m_verbose)
    {
        std::cout << "# INFO: Descriptor computation took " << timeInSeconds() - ts << "s." << std::endl;
    }
}

void
FeatureTracker::matchPointFeaturesWithBestMatchTest(const cv::Mat& dtor1,
                                                    const cv::Mat& dtor2,
                                                    std::vector<std::vector<cv::DMatch> >& matches,
                                                    const cv::Mat& mask)
{
    double ts = timeInSeconds();

    matches.clear();

    std::vector<cv::DMatch> rawMatches;
    m_descriptorMatcher->match(dtor1, dtor2, rawMatches);

    for (size_t i = 0; i < rawMatches.size(); ++i)
    {
        std::vector<cv::DMatch> match;
        match.push_back(rawMatches.at(i));
        matches.push_back(match);
    }

    if (m_verbose)
    {
        std::cout << "# INFO: Descriptor matching took " << timeInSeconds() - ts << "s." << std::endl;
    }
}

void
FeatureTracker::matchPointFeaturesWithRadiusTest(const cv::Mat& dtor1,
                                                 const cv::Mat& dtor2,
                                                 std::vector<std::vector<cv::DMatch> >& matches,
                                                 const cv::Mat& mask,
                                                 float maxDistance)
{
    double ts = timeInSeconds();

    matches.clear();

    std::vector<std::vector<cv::DMatch> > rawMatches;

    switch (m_descriptorType)
    {
    case ORB_DESCRIPTOR:
    case ORB_GPU_DESCRIPTOR:
    {
        if ((m_matchTestType & 0x10) == 0x10)
        {
            m_ORB_GPU->radiusMatch(dtor1, dtor2, rawMatches, maxDistance, mask, true);
        }
        else
        {
            m_descriptorMatcher->radiusMatch(dtor1, dtor2, rawMatches, maxDistance, mask, true);
        }
        break;
    }
    case SURF_DESCRIPTOR:
    case SURF_GPU_DESCRIPTOR:
    {
        if ((m_matchTestType & 0x10) == 0x10)
        {
            m_SURF_GPU->radiusMatch(dtor1, dtor2, rawMatches, maxDistance, mask, true);
        }
        else
        {
            m_descriptorMatcher->radiusMatch(dtor1, dtor2, rawMatches, maxDistance, mask, true);
        }
        break;
    }
    default:
        m_descriptorMatcher->radiusMatch(dtor1, dtor2, rawMatches, maxDistance, mask, true);
    }

    for (size_t i = 0; i < rawMatches.size(); ++i)
    {
        std::vector<cv::DMatch> match;

        for (size_t j = 0; j < rawMatches.at(i).size(); ++j)
        {
            match.push_back(rawMatches.at(i).at(j));
        }

        if (!match.empty())
        {
            matches.push_back(match);
        }
    }

    if (m_verbose)
    {
        std::cout << "# INFO: Descriptor matching took " << timeInSeconds() - ts << "s." << std::endl;
    }
}

void
FeatureTracker::matchPointFeaturesWithRatioTest(const cv::Mat& dtor1,
                                                const cv::Mat& dtor2,
                                                std::vector<std::vector<cv::DMatch> >& matches,
                                                const cv::Mat& mask)
{
    double ts = timeInSeconds();
    size_t knn = 5;
    matches.clear();

    std::vector<std::vector<cv::DMatch> > rawMatches;
    switch (m_descriptorType)
    {
    case ORB_DESCRIPTOR:
    case ORB_GPU_DESCRIPTOR:
    {
        if ((m_matchTestType & 0x10) == 0x10)
        {
            m_ORB_GPU->knnMatch(dtor1, dtor2, rawMatches, knn, mask, true);
        }
        else
        {
            m_descriptorMatcher->knnMatch(dtor1, dtor2, rawMatches, knn, mask, true);
        }
        break;
    }
    case SURF_DESCRIPTOR:
    case SURF_GPU_DESCRIPTOR:
    {
        if ((m_matchTestType & 0x10) == 0x10)
        {
            m_SURF_GPU->knnMatch(dtor1, dtor2, rawMatches, knn, mask, true);
        }
        else
        {
            m_descriptorMatcher->knnMatch(dtor1, dtor2, rawMatches, knn, mask, true);
        }
        break;
    }
    default:
        m_descriptorMatcher->knnMatch(dtor1, dtor2, rawMatches, knn, mask, true);
    }

    for (size_t i = 0; i < rawMatches.size(); ++i)
    {
        std::vector<cv::DMatch>& rawMatch = rawMatches.at(i);

        if (rawMatch.size() < 2)
        {
            continue;
        }

        std::vector<cv::DMatch> match;

        float distanceRatio = rawMatch.at(0).distance / rawMatch.at(1).distance;

        if (distanceRatio < m_maxDistanceRatio)
        {
            match.push_back(rawMatch.at(0));
        }

        if (!match.empty())
        {
            matches.push_back(match);
        }
    }

    if (m_verbose)
    {
        std::cout << "# INFO: Descriptor matching took " << timeInSeconds() - ts << "s." << std::endl;
    }
}

void
FeatureTracker::windowedMatchingMask(const std::vector<cv::KeyPoint>& keypoints1,
                                     const std::vector<cv::KeyPoint>& keypoints2,
                                     float maxDeltaX, float maxDeltaY,
                                     cv::Mat& mask) const
{
    if (keypoints1.empty() || keypoints2.empty())
    {
        mask = cv::Scalar(0);
        return;
    }

    int n1 = static_cast<int>(keypoints1.size());
    int n2 = static_cast<int>(keypoints2.size());

    int rows = std::max(n1, mask.rows);
    int cols = std::max(n2, mask.cols);

    if (rows != mask.rows || cols != mask.cols)
    {
        mask = cv::Mat(rows, cols, CV_8UC1);
    }

    mask = cv::Scalar(0);
    for (int i = 0; i < n1; ++i)
    {
        const cv::Point2f& keypoint1 = keypoints1.at(i).pt;
        for (int j = 0; j < n2; ++j)
        {
            cv::Point2f diff = keypoints2.at(j).pt - keypoint1;

            mask.at<uchar>(i, j) = std::abs(diff.x) < maxDeltaX && std::abs(diff.y) < maxDeltaY;
        }
    }
}

/***************************************************/
/* Temporal Feature Tracker                           */
/***************************************************/

TemporalFeatureTracker::TemporalFeatureTracker(const CameraConstPtr& camera,
                                               DetectorType detectorType,
                                               DescriptorType descriptorType,
                                               MatchTestType matchTestType,
                                               bool preprocess,
                                               const Eigen::Matrix4d& globalCameraPose)
 : FeatureTracker(detectorType, descriptorType, matchTestType, preprocess)
 , k_camera(camera)
 , m_init(false)
 , m_BA(camera, 20, 6, SlidingWindowBA::VO, globalCameraPose)
 , k_maxDelta(80.0f)
 , k_minFeatureCorrespondences(15)
 , k_nominalFocalLength(300.0)
 , k_reprojErrorThresh(1.0)
{

}

bool
TemporalFeatureTracker::addFrame(FramePtr& frame, const cv::Mat& mask)
{
    if (frame->image().channels() > 1)
    {
        cv::cvtColor(frame->image(), m_image, CV_BGR2GRAY);
    }
    else
    {
        frame->image().copyTo(m_image);
    }

    if (mask.empty())
    {
        m_mask = cv::Mat();
    }
    else
    {
        m_mask = mask > 0;
    }

    if (m_preprocess)
    {
        preprocessImage(m_image, m_mask);
    }

    detectFeatures(m_image, m_kpts, m_mask);
    computeDescriptors(m_image, m_kpts, m_dtor);

    if (m_BA.empty())
    {
        m_frames.clear();
        m_poses.clear();
    }

    if (m_BA.windowSize() == 1 && m_poses.size() > 1)
    {
        m_frames.clear();
        m_frames.push_back(m_BA.currentFrame());
        m_poses = m_BA.poses();
    }
    m_BA.setVerbose(m_verbose);

    std::vector<Point2DFeaturePtr> pointFeaturesPrev;
    pointFeaturesPrev.swap(m_pointFeatures);

    for (size_t i = 0; i < m_kpts.size(); ++i)
    {
        Point2DFeaturePtr p = boost::make_shared<Point2DFeature>();
        m_dtor.row(i).copyTo(p->descriptor());
        p->keypoint() = m_kpts.at(i);
        p->index() = i;

        m_pointFeatures.push_back(p);
    }

    if (m_framePrev)
    {
        std::vector<std::vector<cv::DMatch> > matches;

        windowedMatchingMask(m_kpts, m_kptsPrev, k_maxDelta, k_maxDelta, m_matchingMask);

        cv::Mat matchingMask_rOI(m_matchingMask, cv::Rect(0, 0, m_kptsPrev.size(), m_kpts.size()));

        switch (m_matchTestType)
        {
        case BEST_MATCH:
            matchPointFeaturesWithBestMatchTest(m_dtor, m_dtorPrev, matches, matchingMask_rOI);
            break;
        case RADIUS:
            matchPointFeaturesWithRadiusTest(m_dtor, m_dtorPrev, matches, matchingMask_rOI);
            break;
        case RATIO:
        default:
            matchPointFeaturesWithRatioTest(m_dtor, m_dtorPrev, matches, matchingMask_rOI);
        }

        for (size_t i = 0; i < matches.size(); ++i)
        {
            std::vector<cv::DMatch>& match = matches.at(i);
            for (size_t j = 0; j < match.size(); ++j)
            {
                int queryIdx = match.at(j).queryIdx;
                int trainIdx = match.at(j).trainIdx;

                if (queryIdx >= 0 && queryIdx < (int)m_pointFeatures.size() &&
                    trainIdx >= 0 && trainIdx < (int)pointFeaturesPrev.size())
                {
                    m_pointFeatures.at(queryIdx)->prevMatches().push_back(pointFeaturesPrev.at(trainIdx));
                    m_pointFeatures.at(queryIdx)->bestPrevMatchId() = 0;

                    pointFeaturesPrev.at(trainIdx)->nextMatches().push_back(m_pointFeatures.at(queryIdx));
                    pointFeaturesPrev.at(trainIdx)->bestNextMatchId() = 0;
                }
                else
                {
                    if (queryIdx < 0 || queryIdx >= (int)m_pointFeatures.size())
                    {
                        std::cout << "# WARNING: Query idx does not have a valid value " << queryIdx << " " << m_pointFeatures.size() << std::endl;
                    }
                    if (trainIdx < 0 || trainIdx >= (int)pointFeaturesPrev.size())
                    {
                        std::cout << "# WARNING: Train idx does not have a valid value " << trainIdx << " " << pointFeaturesPrev.size() << std::endl;
                    }
                }
            }
        }

        // cross-check
        int invalidMatchCount = 0;

        for (size_t i = 0; i < m_pointFeatures.size(); ++i)
        {
            Point2DFeaturePtr& pf = m_pointFeatures.at(i);
            if (pf->prevMatches().empty() || pf->bestPrevMatchId() == -1)
            {
                continue;
            }

            Point2DFeaturePtr pfPrev = pf->prevMatch().lock();
            if (pfPrev.get() == 0)
            {
                continue;
            }

            if (pfPrev->nextMatches().empty() || pfPrev->bestNextMatchId() == -1)
            {
                pf->bestPrevMatchId() = -1;

                ++invalidMatchCount;

                continue;
            }

            Point2DFeaturePtr nextMatch = pfPrev->nextMatch().lock();
            if (nextMatch.get() == 0 || nextMatch.get() != pf.get())
            {
                pfPrev->bestNextMatchId() = -1;
                pf->bestPrevMatchId() = -1;

                ++invalidMatchCount;
            }
        }

        if (m_verbose)
        {
            std::cout << "# INFO: Removed " << invalidMatchCount << " matches via cross-checking." << std::endl;

            int validMatchCount = 0;
            for (size_t i = 0; i < m_pointFeatures.size(); ++i)
            {
                Point2DFeaturePtr& pf = m_pointFeatures.at(i);
                if (!pf->prevMatches().empty() && pf->bestPrevMatchId() != -1)
                {
                    ++validMatchCount;
                }
            }

            std::cout << "# INFO: # good matches: " << validMatchCount << std::endl;
        }

        // remove singleton features from previous frame
        std::vector<Point2DFeaturePtr>::iterator itF2D = m_framePrev->features2D().begin();

        while (itF2D != m_framePrev->features2D().end())
        {
            Point2DFeaturePtr pf = *itF2D;

            bool hasPrevMatch = (!pf->prevMatches().empty()) && (pf->bestPrevMatchId() != -1);
            bool hasNextMatch = (!pf->nextMatches().empty()) && (pf->bestNextMatchId() != -1);

            if (!hasPrevMatch && !hasNextMatch)
            {
                itF2D = m_framePrev->features2D().erase(itF2D);
            }
            else
            {
                ++itF2D;
            }
        }
    }

    m_kptsPrev = m_kpts;
    m_dtor.copyTo(m_dtorPrev);
    m_framePrev = frame;

    if (!m_image.empty())
    {
        m_image.copyTo(frame->image());
    }

    frame->features2D() = m_pointFeatures;
    for (size_t i = 0; i < m_pointFeatures.size(); ++i)
    {
        frame->features2D().at(i)->frame() = frame;
    }

    if (!m_init)
    {
        m_BA.addFrame(frame);

        m_poses.push_back(frame->cameraPose()->toMatrix());
        m_frames.push_back(m_BA.currentFrame());

        m_init = true;

        return true;
    }

    bool voValid = m_BA.addFrame(frame);
    if (voValid)
    {
        m_poses.push_back(frame->cameraPose()->toMatrix());
        m_frames.push_back(m_BA.currentFrame());

        // update all windowed poses
        std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > window = m_BA.poses();
        for (size_t j = 0; j < window.size(); ++j)
        {
           m_poses.at(m_poses.size() + j - window.size()) = window.at(j);
        }
    }
    else
    {
        m_BA.clear();
        m_BA.addFrame(frame);

        voValid = false;

        if (m_verbose)
        {
            std::cout << "# INFO: Sliding window BA failed." << std::endl;
        }
    }

    if (!voValid)
    {
        for (size_t i = 0; i < m_pointFeatures.size(); ++i)
        {
            m_pointFeatures.at(i)->bestPrevMatchId() = -1;
        }
    }

    cv::cvtColor(m_image, m_sketch, CV_GRAY2BGR);
    cv::drawKeypoints(m_sketch, m_kpts, m_sketch, cv::Scalar(0, 0, 255));

    visualizeTracks();

    return voValid;
}

void
TemporalFeatureTracker::clear(void)
{
    m_init = false;

    m_kpts.clear();
    m_kptsPrev.clear();

    m_dtor = cv::Mat();
    m_dtorPrev = cv::Mat();

    m_pointFeatures.clear();

    m_BA.clear();
    m_frames.clear();
    m_poses.clear();
}

void
TemporalFeatureTracker::runBundleAdjustment(void)
{
    if (m_frames.size() < 2)
    {
        return;
    }

    ceres::Problem problem;

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.max_num_iterations = 1000;

    for (size_t i = 0; i < m_frames.size(); ++i)
    {
        FramePtr& frame = m_frames.at(i);

        std::vector<Point2DFeaturePtr>& features2D = frame->features2D();

        for (size_t j = 0; j < features2D.size(); ++j)
        {
            Point2DFeaturePtr& feature2D = features2D.at(j);
            Point3DFeaturePtr& feature3D = feature2D->feature3D();

            if (!feature2D->feature3D())
            {
                continue;
            }

            ceres::LossFunction* lossFunction = new ceres::CauchyLoss(1.0);

            ceres::CostFunction* costFunction =
                CostFunctionFactory::instance()->generateCostFunction(k_camera,
                                                                      Eigen::Vector2d(feature2D->keypoint().pt.x,
                                                                                      feature2D->keypoint().pt.y),
                                                                      CAMERA_POSE | POINT_3D);

            problem.AddResidualBlock(costFunction, lossFunction,
                                     frame->cameraPose()->rotationData(), frame->cameraPose()->translationData(),
                                     feature3D->pointData());
        }

        ceres::LocalParameterization* quaternionParameterization =
            new ceres::QuaternionParameterization;

        problem.SetParameterization(frame->cameraPose()->rotationData(), quaternionParameterization);
    }

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    if (m_verbose)
    {
        std::cout << summary.BriefReport() << std::endl;
    }
}

void
TemporalFeatureTracker::getMatches(std::vector<cv::Point2f>& matchedPoints,
                                   std::vector<cv::Point2f>& matchedPointsPrev) const
{
    matchedPoints.clear();
    matchedPointsPrev.clear();

    for (size_t i = 0; i < m_pointFeatures.size(); ++i)
    {
        const Point2DFeatureConstPtr& pt = m_pointFeatures.at(i);

        if (pt->prevMatches().empty() || pt->bestPrevMatchId() == -1)
        {
            continue;
        }

        const Point2DFeatureConstPtr ptPrev = pt->prevMatch().lock();

        if (ptPrev.get() != 0)
        {
            matchedPoints.push_back(pt->keypoint().pt);
            matchedPointsPrev.push_back(ptPrev->keypoint().pt);
        }
    }
}

std::vector<FramePtr>&
TemporalFeatureTracker::getFrames(void)
{
    return m_frames;
}

const std::vector<FramePtr>&
TemporalFeatureTracker::getFrames(void) const
{
    return m_frames;
}

const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> >&
TemporalFeatureTracker::getPoses(void) const
{
    return m_poses;
}

std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >
TemporalFeatureTracker::getScenePoints(void) const
{
    return m_BA.scenePoints();
}

void
TemporalFeatureTracker::visualizeTracks(void)
{
    int drawShiftBits = 4;
    int drawMultiplier = 1 << drawShiftBits;

    cv::Scalar green(0, 255, 0);

    for (size_t i = 0; i < m_pointFeatures.size(); ++i)
    {
        std::vector<cv::Point2f> pts;

        Point2DFeaturePtr pt = m_pointFeatures.at(i);
        pts.push_back(pt->keypoint().pt);
        while (!pt->prevMatches().empty() && pt->bestPrevMatchId() != -1)
        {
            pt = pt->prevMatch().lock();

            if (!pt || !pt->feature3D())
            {
                break;
            }

            pts.push_back(pt->keypoint().pt);
        }

        if (pts.size() < 2)
        {
            continue;
        }

        for (size_t j = 0; j < pts.size() - 1; ++j)
        {
            const cv::Point2f& p1 = pts.at(j);
            const cv::Point2f& p2 = pts.at(j + 1);

            if (p1.x < 0.0 || p1.x > m_sketch.cols - 1 ||
                p1.y < 0.0 || p1.y > m_sketch.rows - 1 ||
                p2.x < 0.0 || p2.x > m_sketch.cols - 1 ||
                p2.y < 0.0 || p2.y > m_sketch.rows - 1)
            {
                continue;
            }

            cv::line(m_sketch,
                     cv::Point(cvRound(p1.x * drawMultiplier),
                               cvRound(p1.y * drawMultiplier)),
                     cv::Point(cvRound(p2.x * drawMultiplier),
                               cvRound(p2.y * drawMultiplier)),
                     green, 2, CV_AA, drawShiftBits);
        }
    }
}

}
