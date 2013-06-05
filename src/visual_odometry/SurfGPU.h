#ifndef SURFGPU_H
#define SURFGPU_H

#include <boost/thread/mutex.hpp>
#include <opencv2/nonfree/gpu.hpp>

namespace camodocal
{

class SurfGPU
{
public:
    SurfGPU(double hessianThreshold, int nOctaves=4,
            int nOctaveLayers=2, bool extended=false,
            float keypointsRatio=0.01f);

    ~SurfGPU();

    static cv::Ptr<SurfGPU> instance(double hessianThreshold, int nOctaves=4,
                                     int nOctaveLayers=2, bool extended=false,
                                     float keypointsRatio=0.01f);

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

    void match(const cv::Mat& image1, std::vector<cv::KeyPoint>& keypoints1,
               const cv::Mat& image2, std::vector<cv::KeyPoint>& keypoints2,
               std::vector<cv::DMatch>& matches,
               float maxDistanceRatio = 0.7f);

private:
    static cv::Ptr<SurfGPU> mInstance;

    cv::gpu::SURF_GPU mSURF_GPU;
    cv::gpu::GpuMat mImageGPU;
    cv::gpu::GpuMat mMaskGPU;
    cv::gpu::GpuMat mKptsGPU;
    cv::gpu::GpuMat mDtorsGPU;

    cv::gpu::BruteForceMatcher_GPU<cv::L2<float> > mMatcher;

    cv::gpu::GpuMat mMatchMaskGPU;
    cv::gpu::GpuMat mQDtorsGPU, mTDtorsGPU;

    boost::mutex mSURFMutex;
    boost::mutex mMatchMutex;
};

}

#endif
