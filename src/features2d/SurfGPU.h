#ifndef SURFGPU_H
#define SURFGPU_H

#include <boost/thread.hpp>

#ifdef HAVE_CUDA
#include <opencv2/nonfree/gpu.hpp>
#include <opencv2/gpu/gpu.hpp>
#else
#include <opencv2/nonfree/features2d.hpp>
#endif // HAVE_CUDA

#include <opencv2/legacy/legacy.hpp>

namespace camodocal
{

class SurfGPU
{

#ifdef HAVE_CUDA
    typedef cv::gpu::GpuMat MatType;
#else
    typedef cv::Mat MatType;
#endif // HAVE_CUDA
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
               cv::Mat& dtors1, const cv::Mat& mask1,
               const cv::Mat& image2, std::vector<cv::KeyPoint>& keypoints2,
               cv::Mat& dtors2, const cv::Mat& mask2,
               std::vector<cv::DMatch>& matches,
               bool useProvidedKeypoints = false,
               float maxDistanceRatio = 0.7f);

private:
    static cv::Ptr<SurfGPU> m_instance;
    static boost::mutex m_instanceMutex;

    
#ifdef HAVE_CUDA
    cv::gpu::SURF_GPU m_surfGPU;
    cv::gpu::BruteForceMatcher_GPU<cv::L2<float> > m_matcher;
#else
    cv::SURF m_surfGPU;
    cv::BruteForceMatcher<cv::L2<float> > m_matcher;
#endif // HAVE_CUDA
};

}

#endif
