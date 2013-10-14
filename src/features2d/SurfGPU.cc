#include "SurfGPU.h"

#include <iostream>

namespace camodocal
{

cv::Ptr<SurfGPU> SurfGPU::m_instance;
boost::mutex SurfGPU::m_instanceMutex;

SurfGPU::SurfGPU(double hessianThreshold, int nOctaves,
                 int nOctaveLayers, bool extended,
                 float keypointsRatio)
 : m_surfGPU(hessianThreshold, nOctaves, nOctaveLayers, extended, keypointsRatio)
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

    cv::gpu::GpuMat imageGPU(image);

    cv::gpu::GpuMat maskGPU;
    if (!mask.empty())
    {
        maskGPU.upload(mask);
    }

    try
    {
        cv::gpu::GpuMat kptsGPU;

        m_surfGPU(imageGPU, maskGPU, kptsGPU);

        m_surfGPU.downloadKeypoints(kptsGPU, keypoints);
    }
    catch (cv::Exception& exception)
    {
        std::cout << "# ERROR: Surf GPU feature detection failed: " << exception.msg << std::endl;
    }
}

void
SurfGPU::compute(const cv::Mat& image,
                 std::vector<cv::KeyPoint>& keypoints,
                 cv::Mat& descriptors)
{
    boost::lock_guard<boost::mutex> lock(m_instanceMutex);

    cv::gpu::GpuMat imageGPU(image);

    cv::gpu::GpuMat dtorsGPU;
    try
    {
        m_surfGPU(imageGPU, cv::gpu::GpuMat(), keypoints, dtorsGPU, true);
    }
    catch (cv::Exception& exception)
    {
        std::cout << "# ERROR: Surf GPU descriptor computation failed: " << exception.msg << std::endl;
    }

    dtorsGPU.download(descriptors);
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

    cv::gpu::GpuMat qDtorsGPU(queryDescriptors);
    cv::gpu::GpuMat tDtorsGPU(trainDescriptors);

    cv::gpu::GpuMat maskGPU;
    if (!mask.empty())
    {
        maskGPU.upload(mask);
    }

    m_matcher.knnMatch(qDtorsGPU, tDtorsGPU, matches, k, maskGPU, compactResult);
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

    cv::gpu::GpuMat qDtorsGPU(queryDescriptors);
    cv::gpu::GpuMat tDtorsGPU(trainDescriptors);

    cv::gpu::GpuMat maskGPU;
    if (!mask.empty())
    {
        maskGPU.upload(mask);
    }

    m_matcher.radiusMatch(qDtorsGPU, tDtorsGPU, matches, maxDistance, maskGPU, compactResult);
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

    cv::gpu::GpuMat imageGPU[2];
    cv::gpu::GpuMat maskGPU[2];
    cv::gpu::GpuMat dtorsGPU[2];

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

    try
    {
        m_surfGPU(imageGPU[0], maskGPU[0], keypoints1, dtorsGPU[0], useProvidedKeypoints);
        m_surfGPU(imageGPU[1], maskGPU[1], keypoints2, dtorsGPU[1], useProvidedKeypoints);

        dtorsGPU[0].download(dtors1);
        dtorsGPU[1].download(dtors2);

        std::vector<std::vector<cv::DMatch> > candidateFwdMatches;
        m_matcher.knnMatch(dtorsGPU[0], dtorsGPU[1], candidateFwdMatches, 2);

        std::vector<std::vector<cv::DMatch> > candidateRevMatches;
        m_matcher.knnMatch(dtorsGPU[1], dtorsGPU[0], candidateRevMatches, 2);

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
