#include "SurfGPU.h"

#include <iostream>

namespace camodocal
{

cv::Ptr<SurfGPU> SurfGPU::m_instance;
boost::mutex SurfGPU::m_instanceMutex;

SurfGPU::SurfGPU(double hessianThreshold, int nOctaves,
                 int nOctaveLayers, bool extended,
                 float keypointsRatio)
 :
#ifdef    HAVE_CUDA
#ifdef    HAVE_OPENCV3
    
    // opencv3 + CUDA
    m_matcher(cv::cuda::DescriptorMatcher::createBFMatcher(cv::NORM_L2)),
    m_surfGPU(new SURFType(hessianThreshold,nOctaves,nOctaveLayers,extended,keypointsRatio))
    
#else  // HAVE_OPENCV3
    
    // opencv2 + CUDA
    m_matcher(new MatcherType()),
    m_surfGPU(new SURFType(hessianThreshold, nOctaves, nOctaveLayers, extended, keypointsRatio))
    
#endif // HAVE_OPENCV3
#else  // HAVE_CUDA
#ifdef HAVE_OPENCV3
    
    // opencv3
    m_matcher(cv::DescriptorMatcher::create("BruteForce")),
    m_surfGPU(cv::xfeatures2d::SURF::create(hessianThreshold,nOctaves,nOctaveLayers,extended,keypointsRatio))
    
#else // HAVE_OPENCV3
    
    // opencv2
    m_matcher(new MatcherType()),
    m_surfGPU(new SURFType(hessianThreshold, nOctaves, nOctaveLayers, extended, keypointsRatio))
    
#endif // HAVE_OPENCV3
#endif // HAVE_CUDA
{

}

SurfGPU::~SurfGPU()
{

}

cv::Ptr<SurfGPU>
SurfGPU::instance(double hessianThreshold, int nOctaves,
                  int nOctaveLayers, bool extended,
                  float keypointsRatio)
{
    boost::lock_guard<boost::mutex> lock(m_instanceMutex);

    if (m_instance.empty())
    {
        m_instance = cv::Ptr<SurfGPU>(new SurfGPU(hessianThreshold, nOctaves, nOctaveLayers, extended, keypointsRatio));
    }

    return m_instance;
}

void
SurfGPU::detect(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints,
                const cv::Mat& mask)
{
    boost::lock_guard<boost::mutex> lock(m_instanceMutex);
#ifdef HAVE_CUDA
    // CUDA, Both OpenCV2 and OpenCV3?
    MatType imageGPU(image);

    MatType maskGPU;
    if (!mask.empty())
    {
        maskGPU.upload(mask);
    }

    try
    {
        MatType kptsGPU;

        (*m_surfGPU)(imageGPU, maskGPU, kptsGPU);
        m_surfGPU->downloadKeypoints(kptsGPU, keypoints);
    }
    catch (cv::Exception& exception)
    {
        std::cout << "# ERROR: Surf GPU feature detection failed: " << exception.msg << std::endl;
    }
#else
#ifdef    HAVE_OPENCV3
    m_surfGPU->detect(image,keypoints,mask);
#else  // HAVE_OPENCV3
    (*m_surfGPU)(image,mask,keypoints);
    //m_surfGPU->detect(image,keypoints,mask);
#endif // HAVE_OPENCV3
#endif
}

void
SurfGPU::compute(const cv::Mat& image,
                 std::vector<cv::KeyPoint>& keypoints,
                 cv::Mat& descriptors)
{
    boost::lock_guard<boost::mutex> lock(m_instanceMutex);

#ifdef HAVE_CUDA

    //////////////////
    // OPENCV3.0.0 + OPENCV2 + CUDA
    //////////////////
    
    MatType imageGPU(image);

    MatType dtorsGPU;
    try
    {
        (*m_surfGPU)(imageGPU, MatType(), keypoints, dtorsGPU, true);
    }
    catch (cv::Exception& exception)
    {
        std::cout << "# ERROR: Surf GPU descriptor computation failed: " << exception.msg << std::endl;
    }
    dtorsGPU.download(descriptors);
#else // HAVE_CUDA
#ifdef HAVE_OPENCV3
    //////////////////
    // OPENCV3
    //////////////////
    m_surfGPU->compute(image, keypoints, descriptors);
    
#else // HAVE_OPENCV3

    //////////////////
    // OPENCV2
    //////////////////
    (*m_surfGPU)(image, MatType(), keypoints, descriptors, true);
#endif // HAVE_OPENCV3
#endif // HAVE_CUDA
}

void
SurfGPU::knnMatch(const cv::Mat& queryDescriptors,
                  const cv::Mat& trainDescriptors,
                  std::vector<std::vector<cv::DMatch> >& matches,
                  int k,
                  const cv::Mat& mask,
                  bool compactResult)
{
    boost::lock_guard<boost::mutex> lock(m_instanceMutex);

    if (queryDescriptors.empty() || trainDescriptors.empty())
    {
        matches.clear();
        return;
    }

    matches.reserve(queryDescriptors.rows);
#ifdef HAVE_CUDA
    MatType qDtorsGPU(queryDescriptors);
    MatType tDtorsGPU(trainDescriptors);

    MatType maskGPU;
    if (!mask.empty())
    {
        maskGPU.upload(mask);
    }

    m_matcher->knnMatch(qDtorsGPU, tDtorsGPU, matches, k, maskGPU, compactResult);
#else
    m_matcher->knnMatch(queryDescriptors, trainDescriptors, matches, k,mask,compactResult);
#endif
}

void
SurfGPU::radiusMatch(const cv::Mat& queryDescriptors,
                     const cv::Mat& trainDescriptors,
                     std::vector<std::vector<cv::DMatch> >& matches,
                     float maxDistance,
                     const cv::Mat& mask,
                     bool compactResult)
{
    boost::lock_guard<boost::mutex> lock(m_instanceMutex);

    if (queryDescriptors.empty() || trainDescriptors.empty())
    {
        matches.clear();
        return;
    }

    matches.reserve(queryDescriptors.rows);
#ifdef HAVE_CUDA
    MatType qDtorsGPU(queryDescriptors);
    MatType tDtorsGPU(trainDescriptors);

    MatType maskGPU;
    if (!mask.empty())
    {
        maskGPU.upload(mask);
    }

    m_matcher->radiusMatch(qDtorsGPU, tDtorsGPU, matches, maxDistance, maskGPU, compactResult);
#else
    m_matcher->radiusMatch(queryDescriptors, trainDescriptors, matches, maxDistance, mask, compactResult);
#endif // HAVE_CUDA
}

void
SurfGPU::match(const cv::Mat& image1, std::vector<cv::KeyPoint>& keypoints1,
               cv::Mat& dtors1, const cv::Mat& mask1,
               const cv::Mat& image2, std::vector<cv::KeyPoint>& keypoints2,
               cv::Mat& dtors2, const cv::Mat& mask2,
               std::vector<cv::DMatch>& matches,
               bool useProvidedKeypoints,
               float maxDistanceRatio)
{
    boost::lock_guard<boost::mutex> lock(m_instanceMutex);

    MatType imageGPU[2];
    MatType maskGPU[2];
    MatType dtorsGPU[2];
#ifdef HAVE_CUDA
    imageGPU[0].upload(image1);
    imageGPU[1].upload(image2);

    if (!mask1.empty())
    {
        maskGPU[0].upload(mask1);
    }
    if (!mask2.empty())
    {
        maskGPU[1].upload(mask2);
    }
#else 
/// @todo eliminate the copies here
    imageGPU[0] = image1;
    imageGPU[1] = image2;

    if (!mask1.empty())
    {
        maskGPU[0] = mask1;
    }
    if (!mask2.empty())
    {
        maskGPU[1] = mask2;
    }
#endif
    try
    {
        
#ifdef HAVE_OPENCV3

#ifdef HAVE_CUDA
        // OpenCV3 + CUDA
        // note this is something they missed when implementing OpenCV3, see https://github.com/Itseez/opencv_contrib/issues/280
        (*m_surfGPU)(imageGPU[0], maskGPU[0], keypoints1, dtorsGPU[0], useProvidedKeypoints);
        (*m_surfGPU)(imageGPU[1], maskGPU[1], keypoints2, dtorsGPU[1], useProvidedKeypoints);
#else
        // OpenCV3
        (*m_surfGPU).detectAndCompute(imageGPU[0], maskGPU[0], keypoints1, dtorsGPU[0], useProvidedKeypoints);
        (*m_surfGPU).detectAndCompute(imageGPU[1], maskGPU[1], keypoints2, dtorsGPU[1], useProvidedKeypoints);
#endif
#else
        // OpenCV2
        (*m_surfGPU)(imageGPU[0], maskGPU[0], keypoints1, dtorsGPU[0], useProvidedKeypoints);
        (*m_surfGPU)(imageGPU[1], maskGPU[1], keypoints2, dtorsGPU[1], useProvidedKeypoints);
#endif
        
#ifdef HAVE_CUDA
        dtorsGPU[0].download(dtors1);
        dtorsGPU[1].download(dtors2);
#else
        dtors1 = dtorsGPU[0];
        dtors2 = dtorsGPU[1];
#endif
        std::vector<std::vector<cv::DMatch> > candidateFwdMatches;
        m_matcher->knnMatch(dtorsGPU[0], dtorsGPU[1], candidateFwdMatches, 2);

        std::vector<std::vector<cv::DMatch> > candidateRevMatches;
        m_matcher->knnMatch(dtorsGPU[1], dtorsGPU[0], candidateRevMatches, 2);

        std::vector<std::vector<cv::DMatch> > fwdMatches(candidateFwdMatches.size());
        for (size_t i = 0; i < candidateFwdMatches.size(); ++i)
        {
            std::vector<cv::DMatch>& match = candidateFwdMatches.at(i);

            if (match.size() < 2)
            {
                continue;
            }

            float distanceRatio = match.at(0).distance / match.at(1).distance;

            if (distanceRatio < maxDistanceRatio)
            {
                fwdMatches.at(i).push_back(match.at(0));
            }
        }

        std::vector<std::vector<cv::DMatch> > revMatches(candidateRevMatches.size());
        for (size_t i = 0; i < candidateRevMatches.size(); ++i)
        {
            std::vector<cv::DMatch>& match = candidateRevMatches.at(i);

            if (match.size() < 2)
            {
                continue;
            }

            float distanceRatio = match.at(0).distance / match.at(1).distance;

            if (distanceRatio < maxDistanceRatio)
            {
                revMatches.at(i).push_back(match.at(0));
            }
        }

        // cross-check
        matches.clear();
        for (size_t i = 0; i < fwdMatches.size(); ++i)
        {
            if (fwdMatches.at(i).empty())
            {
                continue;
            }

            cv::DMatch& fwdMatch = fwdMatches.at(i).at(0);

            if (revMatches.at(fwdMatch.trainIdx).empty())
            {
                continue;
            }

            cv::DMatch& revMatch = revMatches.at(fwdMatch.trainIdx).at(0);

            if (fwdMatch.queryIdx == revMatch.trainIdx &&
                fwdMatch.trainIdx == revMatch.queryIdx)
            {
                matches.push_back(fwdMatch);
            }
        }
    }
    catch (cv::Exception& exception)
    {
        std::cout << "# ERROR: Surf GPU image matching failed: " << exception.msg << std::endl;
    }
}

}
