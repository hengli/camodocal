#ifndef ORBGPU_H
#define ORBGPU_H

#include <boost/thread/mutex.hpp>
#include <opencv2/gpu/gpu.hpp>

namespace camodocal
{

class ORBGPU
{
public:
    ORBGPU(int nFeatures = 500, float scaleFactor = 1.2f,
           int nLevels = 8, int edgeThreshold = 31,
           int firstLevel = 0, int WTA_K = 2,
           int scoreType = 0, int patchSize = 31);

    ~ORBGPU();

    static cv::Ptr<ORBGPU> instance(int nFeatures = 500, float scaleFactor = 1.2f,
                                    int nLevels = 8, int edgeThreshold = 31,
                                    int firstLevel = 0, int WTA_K = 2,
                                    int scoreType = 0, int patchSize = 31);

    void detect(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints,
                const cv::Mat& mask = cv::Mat());
    void compute(const cv::Mat& image,
                 std::vector<cv::KeyPoint>& keypoints,
                 cv::Mat& descriptors);
    void knnMatch(const cv::Mat& queryDescriptors, const cv::Mat& trainDescriptors,
                  std::vector<std::vector<cv::DMatch> >& matches, int k,
                  const cv::Mat& mask = cv::Mat(), bool compactResult = false);
    void radiusMatch(const cv::Mat& queryDescriptors, const cv::Mat& trainDescriptors,
                     std::vector<std::vector<cv::DMatch> >& matches, float maxDistance,
                     const cv::Mat& mask = cv::Mat(), bool compactResult = false);

private:
    static cv::Ptr<ORBGPU> mInstance;

    cv::gpu::ORB_GPU mORB_GPU;
    cv::gpu::GpuMat mImageGPU;
    cv::gpu::GpuMat mMaskGPU;
    cv::gpu::GpuMat mKptsGPU;
    cv::gpu::GpuMat mDtorsGPU;

    cv::gpu::BruteForceMatcher_GPU<cv::Hamming> mMatcher;

    cv::gpu::GpuMat mMatchMaskGPU;
    cv::gpu::GpuMat mQDtorsGPU, mTDtorsGPU;

    boost::mutex mORBMutex;
    boost::mutex mMatchMutex;
};

}

#endif
