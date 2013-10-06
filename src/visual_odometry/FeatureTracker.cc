#include <Eigen/Dense>
#include <glibmm.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/features2d.hpp>

#include "../brisk/include/brisk/brisk.h"
#include "../gpl/gpl.h"
#include "../gpl/EigenUtils.h"
#include "../gpl/OpenCVUtils.h"
#include "../npoint/five-point/five-point.hpp"
#include "FeatureTracker.h"

namespace camodocal
{

FeatureTracker::FeatureTracker(DetectorType detectorType,
                               DescriptorType descriptorType,
                               MatchTestType matchTestType,
                               bool preprocess)
 : mMaxDistanceRatio(0.7f)
 , mDetectorType(detectorType)
 , mDescriptorType(descriptorType)
 , mMatchTestType(matchTestType)
 , mPreprocess(preprocess)
 , mVerbose(false)
{
    bool crossCheck = false;

    switch (detectorType)
    {
    case FAST_DETECTOR:
        mFeatureDetector = cv::Ptr<cv::FeatureDetector>(new cv::FastFeatureDetector(25));
        break;
    case ORB_DETECTOR:
        mFeatureDetector = cv::Ptr<cv::FeatureDetector>(new cv::OrbFeatureDetector(1000));
        break;
    case ORB_GPU_DETECTOR:
        mORB_GPU = ORBGPU::instance(1000);
        break;
    case STAR_DETECTOR:
//        mFeatureDetector = new cv::GridAdaptedFeatureDetector(new cv::StarDetector(15, 5, 10, 8, 20), 500, 3, 2);
        mFeatureDetector = cv::Ptr<cv::FeatureDetector>(new cv::StarDetector(16, 25, 10, 8, 5));
        break;
    case SURF_GPU_DETECTOR:
        mSURF_GPU = SurfGPU::instance(200.0);
        break; 
    case SURF_DETECTOR:
    default:
        mFeatureDetector = cv::Ptr<cv::FeatureDetector>(new cv::SurfFeatureDetector(500.0, 5, 2));
    }

    switch (descriptorType)
    {
    case BRISK_DESCRIPTOR:
        mDescriptorExtractor = cv::Ptr<cv::DescriptorExtractor>(new cv::BriskDescriptorExtractor);
        mDescriptorMatcher = cv::Ptr<cv::DescriptorMatcher>(new cv::BFMatcher(cv::NORM_HAMMING, crossCheck));
        break;
    case ORB_DESCRIPTOR:
        mDescriptorExtractor = cv::Ptr<cv::DescriptorExtractor>(new cv::OrbDescriptorExtractor);
        mDescriptorMatcher = cv::Ptr<cv::DescriptorMatcher>(new cv::BFMatcher(cv::NORM_HAMMING, crossCheck));
        break;
    case ORB_GPU_DESCRIPTOR:
        mORB_GPU = ORBGPU::instance(1000);
        break;
    case SURF_GPU_DESCRIPTOR:
        mSURF_GPU = SurfGPU::instance(200.0);
        break;
    case SURF_DESCRIPTOR:
    default:
        mDescriptorExtractor = cv::Ptr<cv::DescriptorExtractor>(new cv::SurfDescriptorExtractor(5, 2));
        mDescriptorMatcher = cv::Ptr<cv::DescriptorMatcher>(new cv::BFMatcher(cv::NORM_L2, crossCheck));
    }
}

cv::Mat&
FeatureTracker::cameraMatrix(void)
{
    return mCameraMatrix;
}

const cv::Mat&
FeatureTracker::getSketch(void) const
{
    return mSketch;
}

void
FeatureTracker::setMaxDistanceRatio(float maxDistanceRatio)
{
    mMaxDistanceRatio = maxDistanceRatio;
}

void
FeatureTracker::setVerbose(bool verbose)
{
    mVerbose = verbose;
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

    switch (mDetectorType)
    {
    case ORB_GPU_DETECTOR:
    {
        mORB_GPU->detect(image, keypoints, mask);
        break;
    }
    case SURF_GPU_DETECTOR:
    {
        mSURF_GPU->detect(image, keypoints, mask);
        break;
    }
    default:
        mFeatureDetector->detect(image, keypoints, mask);
    }

    if (mVerbose)
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

    switch (mDescriptorType)
    {
    case ORB_GPU_DESCRIPTOR:
    {
        mORB_GPU->compute(image, keypoints, descriptors);
        break;
    }
    case SURF_GPU_DESCRIPTOR:
    {
        mSURF_GPU->compute(image, keypoints, descriptors);
        break;
    }
    default:
        mDescriptorExtractor->compute(image, keypoints, descriptors);
    }

    if (mVerbose)
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
    mDescriptorMatcher->match(dtor1, dtor2, rawMatches);

    for (size_t i = 0; i < rawMatches.size(); ++i)
    {
        std::vector<cv::DMatch> match;
        match.push_back(rawMatches.at(i));
        matches.push_back(match);
    }

    if (mVerbose)
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

    switch (mDescriptorType)
    {
    case ORB_DESCRIPTOR:
    case ORB_GPU_DESCRIPTOR:
    {
        if ((mMatchTestType & 0x10) == 0x10)
        {
            mORB_GPU->radiusMatch(dtor1, dtor2, rawMatches, maxDistance, mask, true);
        }
        else
        {
            mDescriptorMatcher->radiusMatch(dtor1, dtor2, rawMatches, maxDistance, mask, true);
        }
        break;
    }
    case SURF_DESCRIPTOR:
    case SURF_GPU_DESCRIPTOR:
    {
        if ((mMatchTestType & 0x10) == 0x10)
        {
            mSURF_GPU->radiusMatch(dtor1, dtor2, rawMatches, maxDistance, mask, true);
        }
        else
        {
            mDescriptorMatcher->radiusMatch(dtor1, dtor2, rawMatches, maxDistance, mask, true);
        }
        break;
    }
    default:
        mDescriptorMatcher->radiusMatch(dtor1, dtor2, rawMatches, maxDistance, mask, true);
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

    if (mVerbose)
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
    switch (mDescriptorType)
    {
    case ORB_DESCRIPTOR:
    case ORB_GPU_DESCRIPTOR:
    {
        if ((mMatchTestType & 0x10) == 0x10)
        {
            mORB_GPU->knnMatch(dtor1, dtor2, rawMatches, knn, mask, true);
        }
        else
        {
            mDescriptorMatcher->knnMatch(dtor1, dtor2, rawMatches, knn, mask, true);
        }
        break;
    }
    case SURF_DESCRIPTOR:
    case SURF_GPU_DESCRIPTOR:
    {
        if ((mMatchTestType & 0x10) == 0x10)
        {
            mSURF_GPU->knnMatch(dtor1, dtor2, rawMatches, knn, mask, true);
        }
        else
        {
            mDescriptorMatcher->knnMatch(dtor1, dtor2, rawMatches, knn, mask, true);
        }
        break;
    }
    default:
        mDescriptorMatcher->knnMatch(dtor1, dtor2, rawMatches, knn, mask, true);
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

        if (distanceRatio < mMaxDistanceRatio)
        {
            match.push_back(rawMatch.at(0));
        }

        if (!match.empty())
        {
            matches.push_back(match);
        }
    }

    if (mVerbose)
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
                                               bool preprocess)
 : FeatureTracker(detectorType, descriptorType, matchTestType, preprocess)
 , kCamera(camera)
 , mInit(false)
 , m_BA(camera)
 , kMaxDelta(80.0f)
 , kMinFeatureCorrespondences(15)
 , kNominalFocalLength(300.0)
 , kReprojErrorThresh(1.0)
{

}

bool
TemporalFeatureTracker::addFrame(FramePtr& frame, const cv::Mat& mask,
                                 Eigen::Matrix3d& R_rel, Eigen::Vector3d& t_rel)
{
    if (frame->image().channels() > 1)
    {
        cv::cvtColor(frame->image(), mImage, CV_BGR2GRAY);
    }
    else
    {
        frame->image().copyTo(mImage);
    }

    if (mask.empty())
    {
        mMask = cv::Mat();
    }
    else
    {
        mMask = mask > 0;
    }

    if (mPreprocess)
    {
        preprocessImage(mImage, mMask);
    }

    detectFeatures(mImage, mKpts, mMask);
    computeDescriptors(mImage, mKpts, mDtor);

    if (m_BA.empty())
    {
        mFrames.clear();
        mPoses.clear();
    }

    if (m_BA.windowSize() == 1 && mPoses.size() > 1)
    {
        mFrames.clear();
        mFrames.push_back(m_BA.currentFrame());
        mPoses = m_BA.poses();
    }
    m_BA.setVerbose(mVerbose);

    std::vector<Point2DFeaturePtr> pointFeaturesPrev;
    pointFeaturesPrev.swap(mPointFeatures);

    for (size_t i = 0; i < mKpts.size(); ++i)
    {
        Point2DFeaturePtr p(new Point2DFeature);
        mDtor.row(i).copyTo(p->descriptor());
        p->keypoint() = mKpts.at(i);
        p->index() = i;

        mPointFeatures.push_back(p);
    }

    if (!mDtorPrev.empty())
    {
        std::vector<std::vector<cv::DMatch> > matches;

        windowedMatchingMask(mKpts, mKptsPrev, kMaxDelta, kMaxDelta, mMatchingMask);

        cv::Mat matchingMaskROI(mMatchingMask, cv::Rect(0, 0, mKptsPrev.size(), mKpts.size()));

        switch (mMatchTestType)
        {
        case BEST_MATCH:
            matchPointFeaturesWithBestMatchTest(mDtor, mDtorPrev, matches, matchingMaskROI);
            break;
        case RADIUS:
            matchPointFeaturesWithRadiusTest(mDtor, mDtorPrev, matches, matchingMaskROI);
            break;
        case RATIO:
        default:
            matchPointFeaturesWithRatioTest(mDtor, mDtorPrev, matches, matchingMaskROI);
        }

        for (size_t i = 0; i < matches.size(); ++i)
        {
            std::vector<cv::DMatch>& match = matches.at(i);
            for (size_t j = 0; j < match.size(); ++j)
            {
                int queryIdx = match.at(j).queryIdx;
                int trainIdx = match.at(j).trainIdx;

                if (queryIdx >= 0 && queryIdx < mPointFeatures.size() &&
                    trainIdx >= 0 && trainIdx < pointFeaturesPrev.size())
                {
                    mPointFeatures.at(queryIdx)->prevMatches().push_back(pointFeaturesPrev.at(trainIdx));
                    mPointFeatures.at(queryIdx)->bestPrevMatchId() = 0;

                    pointFeaturesPrev.at(trainIdx)->nextMatches().push_back(mPointFeatures.at(queryIdx));
                    pointFeaturesPrev.at(trainIdx)->bestNextMatchId() = 0;
                }
                else
                {
                    if (queryIdx < 0 || queryIdx >= mPointFeatures.size())
                    {
                        std::cout << "# WARNING: Query idx does not have a valid value " << queryIdx << " " << mPointFeatures.size() << std::endl;
                    }
                    if (trainIdx < 0 || trainIdx >= pointFeaturesPrev.size())
                    {
                        std::cout << "# WARNING: Train idx does not have a valid value " << trainIdx << " " << pointFeaturesPrev.size() << std::endl;
                    }
                }
            }
        }

        // cross-check
        int invalidMatchCount = 0;

        for (size_t i = 0; i < mPointFeatures.size(); ++i)
        {
            Point2DFeaturePtr& pf = mPointFeatures.at(i);
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

        if (mVerbose)
        {
            std::cout << "# INFO: Removed " << invalidMatchCount << " matches via cross-checking." << std::endl;

            int validMatchCount = 0;
            for (size_t i = 0; i < mPointFeatures.size(); ++i)
            {
                Point2DFeaturePtr& pf = mPointFeatures.at(i);
                if (!pf->prevMatches().empty() && pf->bestPrevMatchId() != -1)
                {
                    ++validMatchCount;
                }
            }

            std::cout << "# INFO: # good matches: " << validMatchCount << std::endl;
        }
    }

    mKptsPrev = mKpts;
    mDtor.copyTo(mDtorPrev);

    if (!mImage.empty())
    {
        mImage.copyTo(frame->image());
    }

    frame->features2D() = mPointFeatures;
    for (size_t i = 0; i < mPointFeatures.size(); ++i)
    {
        frame->features2D().at(i)->frame() = frame;
    }

    if (!mInit)
    {
        Eigen::Matrix3d R_abs;
        Eigen::Vector3d t_abs;
        m_BA.addFrame(frame,
                      Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(),
                      R_abs, t_abs);

        mPoses.push_back(homogeneousTransform(R_abs, t_abs));
        mFrames.push_back(m_BA.currentFrame());

        R_rel = R_abs;
        t_rel = t_abs;

        mInit = true;

        return true;
    }

    cv::Mat inliers;
    bool voValid = computeVO(R_rel, t_rel, inliers);

    if (voValid)
    {
        // mark inliers from VO
        int mark = -1;
        for (size_t i = 0; i < mPointFeatures.size(); ++i)
        {
            Point2DFeaturePtr& pt = mPointFeatures.at(i);

            if (pt->prevMatches().empty() || pt->bestPrevMatchId() == -1)
            {
                continue;
            }

            ++mark;

            if (inliers.at<unsigned char>(0, mark) == 0)
            {
                if (Point2DFeaturePtr prevMatch = pt->prevMatch().lock())
                {
                    if (!prevMatch->nextMatches().empty() &&
                        prevMatch->bestNextMatchId() != -1)
                    {
                        prevMatch->bestNextMatchId() = -1;
                    }
                }
                pt->bestPrevMatchId() = -1;
            }
        }

        // TODO: Debug findInliers
//        // find more inliers from feature matches
//        int inlierCount = findInliers(R_rel, t_rel, reprojErrThresh);
//
//        if (mVerbose)
//        {
//            std::cout << "# INFO: Inlier count (" << inliers.total() << "): "
//                      << cv::countNonZero(inliers) << " -> " << inlierCount << std::endl;
//        }

        if (mVerbose)
        {
            std::cout << "# INFO: Inlier count: "
                      << cv::countNonZero(inliers) << "/" << inliers.total() << std::endl;
        }

        if (mVerbose)
        {
            std::cout << "# INFO: VO before optimization: " << std::endl << R_rel << std::endl << t_rel.transpose() << std::endl;
        }

        Eigen::Matrix3d R_abs;
        Eigen::Vector3d t_abs;

        if (m_BA.addFrame(frame, R_rel, t_rel, R_abs, t_abs))
        {
            mPoses.push_back(homogeneousTransform(R_abs, t_abs));
            mFrames.push_back(m_BA.currentFrame());

            // update all windowed poses
            std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > window = m_BA.poses();
            for (size_t j = 0; j < window.size(); ++j)
            {
               mPoses.at(mPoses.size() + j - window.size()) = window.at(j);
            }

            Eigen::Matrix4d H_rel = mPoses.back() * mPoses.at(mPoses.size() - 2).inverse();

            R_rel = H_rel.block<3,3>(0,0);
            t_rel = H_rel.block<3,1>(0,3);

            if (mVerbose)
            {
                std::cout << "# INFO: VO after optimization: " << std::endl << R_rel << std::endl << t_rel.transpose() << std::endl;
            }
        }
        else
        {
            m_BA.clear();
            m_BA.addFrame(frame, R_rel, t_rel, R_abs, t_abs);

            voValid = false;

            if (mVerbose)
            {
                std::cout << "# INFO: Sliding window BA failed." << std::endl;
            }
        }
    }
    else
    {
        Eigen::Matrix3d R_abs;
        Eigen::Vector3d t_abs;

        m_BA.clear();
        m_BA.addFrame(frame, R_rel, t_rel, R_abs, t_abs);
    }

    if (!voValid)
    {
        for (size_t i = 0; i < mPointFeatures.size(); ++i)
        {
            mPointFeatures.at(i)->bestPrevMatchId() = -1;
        }
    }

    cv::cvtColor(mImage, mSketch, CV_GRAY2BGR);
    cv::drawKeypoints(mSketch, mKpts, mSketch, cv::Scalar(0, 0, 255));

    visualizeTracks();

    return voValid;
}

void
TemporalFeatureTracker::clear(void)
{
    mInit = false;

    mKpts.clear();
    mKptsPrev.clear();

    mDtor = cv::Mat();
    mDtorPrev = cv::Mat();

    mPointFeatures.clear();

    m_BA.clear();
    mFrames.clear();
    mPoses.clear();
}

void
TemporalFeatureTracker::getMatches(std::vector<cv::Point2f>& matchedPoints,
                                   std::vector<cv::Point2f>& matchedPointsPrev) const
{
    matchedPoints.clear();
    matchedPointsPrev.clear();

    for (size_t i = 0; i < mPointFeatures.size(); ++i)
    {
        const Point2DFeatureConstPtr& pt = mPointFeatures.at(i);

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
    return mFrames;
}

const std::vector<FramePtr>&
TemporalFeatureTracker::getFrames(void) const
{
    return mFrames;
}

const std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> >&
TemporalFeatureTracker::getPoses(void) const
{
    return mPoses;
}

std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >
TemporalFeatureTracker::getScenePoints(void) const
{
    return m_BA.scenePoints();
}

bool
TemporalFeatureTracker::computeVO(Eigen::Matrix3d& R_rel, Eigen::Vector3d& t_rel,
                                  cv::Mat& inliers)
{
    std::vector<cv::Point2f> pointsPrev, points;
    pointsPrev.reserve(mPointFeatures.size());
    points.reserve(mPointFeatures.size());

    for (size_t i = 0; i < mPointFeatures.size(); ++i)
    {
        const Point2DFeatureConstPtr& pt = mPointFeatures.at(i);
        if (pt->prevMatches().empty() || pt->bestPrevMatchId() == -1)
        {
            continue;
        }

        const Point2DFeatureConstPtr ptPrev = pt->prevMatch().lock();
        if (ptPrev.get() == 0)
        {
            continue;
        }

        cv::Point2f rectPt, rectPtPrev;
        rectifyImagePoint(pt->keypoint().pt, rectPt);
        rectifyImagePoint(ptPrev->keypoint().pt, rectPtPrev);

        points.push_back(rectPt);
        pointsPrev.push_back(rectPtPrev);
    }

    if (points.size() < kMinFeatureCorrespondences)
    {
        return false;
    }

    cv::Mat E, R_rel_cv, t_rel_cv;
    E = findEssentialMat(pointsPrev, points, 1.0, cv::Point2d(0.0, 0.0), CV_FM_RANSAC, 0.99, kReprojErrorThresh / kNominalFocalLength, 1000, inliers);
    recoverPose(E, pointsPrev, points, R_rel_cv, t_rel_cv, 1.0, cv::Point2d(0.0, 0.0), inliers);

    if (cv::countNonZero(inliers) < kMinFeatureCorrespondences)
    {
        return false;
    }

    cv::cv2eigen(R_rel_cv, R_rel);
    cv::cv2eigen(t_rel_cv, t_rel);

    return true;
}

int
TemporalFeatureTracker::findInliers(const Eigen::Matrix3d& R_rel,
                                    const Eigen::Vector3d& t_rel,
                                    double reprojErrorThresh)
{
    int inlierCount = 0;

    double focal = mCameraMatrix.at<double>(0,0);
    Eigen::Vector2d pp(mCameraMatrix.at<double>(0,2), mCameraMatrix.at<double>(1,2));

    Eigen::Matrix3d cameraMatrix;
    cv::cv2eigen(mCameraMatrix, cameraMatrix);

    Eigen::Matrix3d cameraMatrixInv = cameraMatrix.inverse();

    for (size_t i = 0; i < mPointFeatures.size(); ++i)
    {
        Point2DFeaturePtr& pt = mPointFeatures.at(i);

        if (pt->prevMatches().empty())
        {
            continue;
        }

        pt->bestPrevMatchId() = -1;

        cv::Point2f& p2_cv = pt->keypoint().pt;

        Eigen::Vector3d p2;
        p2 << p2_cv.x, p2_cv.y, 1.0;
        p2 = cameraMatrixInv * p2;

        double reprojErrorMin = std::numeric_limits<double>::max();
        for (size_t j = 0; j < pt->prevMatches().size(); ++j)
        {
            Point2DFeaturePtr ptPrev = pt->prevMatches().at(j).lock();
            if (ptPrev.get() == 0)
            {
                continue;
            }

            cv::Point2f& p1_cv = ptPrev->keypoint().pt;

            Eigen::Vector3d p1;
            p1 << p1_cv.x, p1_cv.y, 1.0;
            p1 = cameraMatrixInv * p1;

            double reprojError = sqrt(sampsonError(R_rel, t_rel, p1, p2)) * focal;
            if (reprojError < reprojErrorThresh && reprojError < reprojErrorMin)
            {
                pt->bestPrevMatchId() = j;

                reprojErrorMin = reprojError;
            }
        }

        if (pt->bestPrevMatchId() != -1)
        {
            ++inlierCount;
        }
    }

    return inlierCount;
}

void
TemporalFeatureTracker::rectifyImagePoint(const cv::Point2f& src, cv::Point2f& dst) const
{
    Eigen::Vector3d P;

    kCamera->liftProjective(Eigen::Vector2d(src.x, src.y), P);

    P /= P(2);

    dst.x = P(0);
    dst.y = P(1);
}

void
TemporalFeatureTracker::visualizeTracks(void)
{
    int drawShiftBits = 4;
    int drawMultiplier = 1 << drawShiftBits;

    cv::Scalar green(0, 255, 0);

    for (size_t i = 0; i < mPointFeatures.size(); ++i)
    {
        std::vector<cv::Point2f> pts;

        Point2DFeaturePtr pt = mPointFeatures.at(i);
        pts.push_back(pt->keypoint().pt);
        while (!pt->prevMatches().empty() && pt->bestPrevMatchId() != -1)
        {
            pt = pt->prevMatch().lock();

            if (pt.get() == 0)
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

            if (p1.x < 0.0 || p1.x > mSketch.cols - 1 ||
                p1.y < 0.0 || p1.y > mSketch.rows - 1 ||
                p2.x < 0.0 || p2.x > mSketch.cols - 1 ||
                p2.y < 0.0 || p2.y > mSketch.rows - 1)
            {
                continue;
            }

            cv::line(mSketch,
                     cv::Point(cvRound(p1.x * drawMultiplier),
                               cvRound(p1.y * drawMultiplier)),
                     cv::Point(cvRound(p2.x * drawMultiplier),
                               cvRound(p2.y * drawMultiplier)),
                     green, 2, CV_AA, drawShiftBits);
        }
    }
}

/***************************************************/
/* Camera Rig Temporal Feature Tracker                           */
/***************************************************/

CameraRigTemporalFeatureTracker::CameraRigTemporalFeatureTracker(const std::vector<CameraConstPtr>& cameras,
                                                                 DetectorType detectorType,
                                                                 DescriptorType descriptorType,
                                                                 MatchTestType matchTestType,
                                                                 bool preprocess)
 : FeatureTracker(detectorType, descriptorType, matchTestType, preprocess)
 , kMaxDelta(200.0f)
 , kMinFeatureCorrespondences(15)
 , kNominalFocalLength(300.0)
 , kReprojErrorThresh(4.0)
{
    int nCameras = cameras.size();

    for (int i = 0; i < nCameras; ++i)
    {
        mCameraMetadata.push_back(CameraMetadata(cameras.at(i)));
    }
}

bool
CameraRigTemporalFeatureTracker::addFrame(const std::vector<cv::Mat>& images,
                                          const std::vector<cv::Mat>& masks)
{
    Glib::Threads::Thread* threads[mCameraMetadata.size()];

    for (size_t i = 0; i < mCameraMetadata.size(); ++i)
    {
        CameraMetadata& metadata = mCameraMetadata.at(i);

        threads[i] = Glib::Threads::Thread::create(sigc::bind(sigc::mem_fun(*this, &CameraRigTemporalFeatureTracker::processImage), images.at(i), masks.at(i), &metadata));
    }

    for (size_t i = 0; i < mCameraMetadata.size(); ++i)
    {
        threads[i]->join();
    }

    std::vector<std::vector<Point2DFeaturePtr> > pointFeatures(mCameraMetadata.size());
    for (size_t i = 0; i < mCameraMetadata.size(); ++i)
    {
        pointFeatures.at(i) = mCameraMetadata.at(i).pointFeatures;
    }

    visualizeTracks();

    return true;
}

void
CameraRigTemporalFeatureTracker::clear(void)
{
    mCameraMetadata.clear();
}

const cv::Mat&
CameraRigTemporalFeatureTracker::getSketch(int idx) const
{
    return mCameraMetadata.at(idx).sketch;
}

void
CameraRigTemporalFeatureTracker::processImage(const cv::Mat& image, const cv::Mat& mask,
                                              CameraMetadata* metadata)
{
    if (image.channels() > 1)
    {
        cv::cvtColor(image, metadata->image, CV_BGR2GRAY);
    }
    else
    {
        image.copyTo(metadata->image);
    }

    if (mask.empty())
    {
        metadata->mask = cv::Mat();
    }
    else
    {
        metadata->mask = mask > 0;
    }

    if (mPreprocess)
    {
        preprocessImage(metadata->image, metadata->mask);
    }

    detectFeatures(metadata->image, metadata->kpts, metadata->mask);
    computeDescriptors(metadata->image, metadata->kpts, metadata->dtor);

    std::vector<Point2DFeaturePtr> pointFeaturesPrev;
    pointFeaturesPrev.swap(metadata->pointFeatures);

    std::vector<Point2DFeaturePtr> pointFeatures;

    for (size_t j = 0; j < metadata->kpts.size(); ++j)
    {
        Point2DFeaturePtr p(new Point2DFeature);
        metadata->dtor.row(j).copyTo(p->descriptor());
        p->keypoint() = metadata->kpts.at(j);
        p->index() = j;

        pointFeatures.push_back(p);
    }

    if (!metadata->dtorPrev.empty())
    {
        std::vector<std::vector<cv::DMatch> > matches;

        windowedMatchingMask(metadata->kpts, metadata->kptsPrev, kMaxDelta, kMaxDelta, mMatchingMask);

        cv::Mat matchingMaskROI(mMatchingMask, cv::Rect(0, 0, metadata->kptsPrev.size(), metadata->kpts.size()));

        switch (mMatchTestType)
        {
        case BEST_MATCH:
            matchPointFeaturesWithBestMatchTest(metadata->dtor, metadata->dtorPrev, matches, matchingMaskROI);
            break;
        case RADIUS:
            matchPointFeaturesWithRadiusTest(metadata->dtor, metadata->dtorPrev, matches, matchingMaskROI);
            break;
        case RATIO:
        default:
            matchPointFeaturesWithRatioTest(metadata->dtor, metadata->dtorPrev, matches, matchingMaskROI);
        }

        for (size_t j = 0; j < matches.size(); ++j)
        {
            std::vector<cv::DMatch>& match = matches.at(j);
            for (size_t k = 0; k < match.size(); ++k)
            {
                int queryIdx = match.at(k).queryIdx;
                int trainIdx = match.at(k).trainIdx;

                if (queryIdx >= 0 && queryIdx < pointFeatures.size() &&
                    trainIdx >= 0 && trainIdx < pointFeaturesPrev.size())
                {
                    pointFeatures.at(queryIdx)->prevMatches().push_back(pointFeaturesPrev.at(trainIdx));
                    pointFeatures.at(queryIdx)->bestPrevMatchId() = 0;

                    pointFeaturesPrev.at(trainIdx)->nextMatches().push_back(pointFeatures.at(queryIdx));
                    pointFeaturesPrev.at(trainIdx)->bestNextMatchId() = 0;
                }
                else
                {
                    if (queryIdx < 0 || queryIdx >= pointFeatures.size())
                    {
                        std::cout << "# WARNING: Query idx does not have a valid value " << queryIdx << " " << pointFeatures.size() << std::endl;
                    }
                    if (trainIdx < 0 || trainIdx >= pointFeaturesPrev.size())
                    {
                        std::cout << "# WARNING: Train idx does not have a valid value " << trainIdx << " " << pointFeaturesPrev.size() << std::endl;
                    }
                }
            }
        }

        // cross-check
        int invalidMatchCount = 0;

        for (size_t i = 0; i < pointFeatures.size(); ++i)
        {
            Point2DFeaturePtr& pf = pointFeatures.at(i);
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
                pf->bestPrevMatchId() = -1;

                ++invalidMatchCount;
            }
        }

//        // filter out matches not satisfying epipolar constraint
//        Eigen::Matrix4d H_rel_odo = odoPose.inverse() * mOdoPosePrev;
//        Eigen::Matrix4d H_rel_cam = H_odo_cam * H_rel_odo * H_odo_cam.inverse();
//
//        Eigen::Vector2d pp(metadata->cameraParameters.imageWidth() / 2.0,
//                           metadata->cameraParameters.imageHeight() / 2.0);
//
//        for (size_t i = 0; i < pointFeatures.size(); ++i)
//        {
//            Point2DFeaturePtr& pf = pointFeatures.at(i);
//            if (pf->prevMatches().empty() || pf->bestPrevMatchId() == -1)
//            {
//                continue;
//            }
//
//            Point2DFeaturePtr& pfPrev = pf->prevMatch();
//
//            cv::Point2f rectPtPrev_cv, rectPt_cv;
//            rectifyImagePoint(metadata->camera, metadata->cameraParameters,
//                              pfPrev->keypoint().pt, rectPtPrev_cv);
//            rectifyImagePoint(metadata->camera, metadata->cameraParameters,
//                              pf->keypoint().pt, rectPt_cv);
//
//            Eigen::Vector3d rectPtPrev;
//            rectPtPrev(0) = (rectPtPrev_cv.x - pp(0)) / kNominalFocalLength;
//            rectPtPrev(1) = (rectPtPrev_cv.y - pp(1)) / kNominalFocalLength;
//            rectPtPrev(2) = 1.0;
//
//            Eigen::Vector3d rectPt;
//            rectPt(0) = (rectPt_cv.x - pp(0)) / kNominalFocalLength;
//            rectPt(1) = (rectPt_cv.y - pp(1)) / kNominalFocalLength;
//            rectPt(2) = 1.0;
//
//            double reprojErr = sqrt(sampsonError(H_rel_cam, rectPtPrev, rectPt)) * kNominalFocalLength;
//
//            if (reprojErr > kReprojErrorThresh)
//            {
//                pf->prevMatches().clear();
//                pf->bestPrevMatchId() = -1;
//
//                ++invalidMatchCount;
//            }
//        }

        if (mVerbose)
        {
            std::cout << "# INFO: Removed " << invalidMatchCount << " matches via cross-checking." << std::endl;

            int validMatchCount = 0;
            for (size_t i = 0; i < pointFeatures.size(); ++i)
            {
                Point2DFeaturePtr& pf = pointFeatures.at(i);
                if (!pf->prevMatches().empty() && pf->bestPrevMatchId() != -1)
                {
                    ++validMatchCount;
                }
            }

            std::cout << "# INFO: # good matches: " << validMatchCount << std::endl;
        }
    }

    metadata->pointFeatures = pointFeatures;

    metadata->kptsPrev = metadata->kpts;
    metadata->dtor.copyTo(metadata->dtorPrev);

    cv::cvtColor(metadata->image, metadata->sketch, CV_GRAY2BGR);
    cv::drawKeypoints(metadata->sketch, metadata->kpts, metadata->sketch, cv::Scalar(0, 0, 255));
}

void
CameraRigTemporalFeatureTracker::rectifyImagePoint(const CameraConstPtr& camera,
                                                   const cv::Point2f& src, cv::Point2f& dst) const
{
    Eigen::Vector3d P;

    camera->liftProjective(Eigen::Vector2d(src.x, src.y), P);

    P /= P(2);

    dst.x = P(0);
    dst.y = P(1);
}

void
CameraRigTemporalFeatureTracker::visualizeTracks(void)
{
    int drawShiftBits = 4;
    int drawMultiplier = 1 << drawShiftBits;

    cv::Scalar green(0, 255, 0);

    for (size_t i = 0; i < mCameraMetadata.size(); ++i)
    {
        CameraMetadata& metadata = mCameraMetadata.at(i);

        for (size_t j = 0; j < metadata.pointFeatures.size(); ++j)
        {
            std::vector<cv::Point2f> pts;

            Point2DFeaturePtr pt = metadata.pointFeatures.at(j);
            pts.push_back(pt->keypoint().pt);
            while (!pt->prevMatches().empty() && pt->bestPrevMatchId() != -1)
            {
                pt = pt->prevMatches().at(pt->bestPrevMatchId()).lock();

                if (pt.get() == 0)
                {
                    break;
                }

                pts.push_back(pt->keypoint().pt);
            }

            if (pts.size() < 2)
            {
                continue;
            }

            for (size_t k = 0; k < pts.size() - 1; ++k)
            {
                const cv::Point2f& p1 = pts.at(k);
                const cv::Point2f& p2 = pts.at(k + 1);

                cv::line(metadata.sketch,
                         cv::Point(cvRound(p1.x * drawMultiplier),
                                   cvRound(p1.y * drawMultiplier)),
                         cv::Point(cvRound(p2.x * drawMultiplier),
                                   cvRound(p2.y * drawMultiplier)),
                         green, 2, CV_AA, drawShiftBits);
            }
        }
    }
}

}
