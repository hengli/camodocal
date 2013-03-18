#include "ORBGPU.h"

#include <iostream>

namespace camodocal
{

cv::Ptr<ORBGPU> ORBGPU::mInstance;

ORBGPU::ORBGPU(int nFeatures, float scaleFactor,
               int nLevels, int edgeThreshold,
               int firstLevel, int WTA_K,
               int scoreType, int patchSize)
 : mORB_GPU(nFeatures, scaleFactor, nLevels, edgeThreshold,
            firstLevel, WTA_K, scoreType, patchSize)
{

}

ORBGPU::~ORBGPU()
{

}

cv::Ptr<ORBGPU>
ORBGPU::instance(int nFeatures, float scaleFactor,
                 int nLevels, int edgeThreshold,
                 int firstLevel, int WTA_K,
                 int scoreType, int patchSize)
{
    if (mInstance.empty())
    {
        mInstance = cv::Ptr<ORBGPU>(new ORBGPU(nFeatures, scaleFactor,
                                               nLevels, edgeThreshold,
                                               firstLevel, WTA_K,
                                               scoreType, patchSize));
    }

    return mInstance;
}

void
ORBGPU::detect(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints,
               const cv::Mat& mask)
{
    boost::mutex::scoped_lock lock(mORBMutex);

    mImageGPU.upload(image);
    if (!mask.empty())
    {
        mMaskGPU.upload(mask);
    }
    else
    {
        mMaskGPU.release();
    }

    try
    {
        mORB_GPU(mImageGPU, mMaskGPU, mKptsGPU);

        mORB_GPU.downloadKeyPoints(mKptsGPU, keypoints);
    }
    catch (cv::Exception& exception)
    {
        std::cout << "# ERROR: ORB GPU feature detection failed: " << exception.msg << std::endl;
    }

    mImageGPU.release();
    mMaskGPU.release();
    mKptsGPU.release();
}

void
ORBGPU::compute(const cv::Mat& image,
                 std::vector<cv::KeyPoint>& keypoints,
                 cv::Mat& descriptors)
{
    boost::mutex::scoped_lock lock(mORBMutex);

    mImageGPU.upload(image);

    try
    {
        mORB_GPU(mImageGPU, cv::gpu::GpuMat(), keypoints, mDtorsGPU);
    }
    catch (cv::Exception& exception)
    {
        std::cout << "# ERROR: ORB GPU descriptor computation failed: " << exception.msg << std::endl;
    }

    mDtorsGPU.download(descriptors);

    mImageGPU.release();
    mDtorsGPU.release();
}

void
ORBGPU::knnMatch(const cv::Mat& queryDescriptors,
                  const cv::Mat& trainDescriptors,
                  std::vector<std::vector<cv::DMatch> >& matches,
                  int k,
                  const cv::Mat& mask,
                  bool compactResult)
{
    boost::mutex::scoped_lock lock(mMatchMutex);

    if (queryDescriptors.empty() || trainDescriptors.empty())
    {
        matches.clear();
        return;
    }

    matches.reserve(queryDescriptors.rows);

    mQDtorsGPU.upload(queryDescriptors);
    mTDtorsGPU.upload(trainDescriptors);

    if (!mask.empty())
    {
        mMatchMaskGPU.upload(mask);
    }
    else
    {
        mMatchMaskGPU.release();
    }

    mMatcher.knnMatch(mQDtorsGPU, mTDtorsGPU, matches, k, mMatchMaskGPU, compactResult);
}

void
ORBGPU::radiusMatch(const cv::Mat& queryDescriptors,
                     const cv::Mat& trainDescriptors,
                     std::vector<std::vector<cv::DMatch> >& matches,
                     float maxDistance,
                     const cv::Mat& mask,
                     bool compactResult)
{
    boost::mutex::scoped_lock lock(mMatchMutex);

    if (queryDescriptors.empty() || trainDescriptors.empty())
    {
        matches.clear();
        return;
    }

    matches.reserve(queryDescriptors.rows);

    mQDtorsGPU.upload(queryDescriptors);
    mTDtorsGPU.upload(trainDescriptors);

    if (!mask.empty())
    {
        mMatchMaskGPU.upload(mask);
    }
    else
    {
        mMatchMaskGPU.release();
    }

    mMatcher.radiusMatch(mQDtorsGPU, mTDtorsGPU, matches, maxDistance, mMatchMaskGPU, compactResult);
}

}
