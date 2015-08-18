#ifndef ORBGPU_H
#define ORBGPU_H

#include <boost/thread/mutex.hpp>


#ifdef HAVE_CUDA
#ifdef HAVE_OPENCV3

    //////////////////
    // CUDA + OPENCV3
    //////////////////
#include <opencv2/cudafeatures2d.hpp>
#include <opencv2/features2d.hpp>

#else  // HAVE_OPENCV3

    //////////////////
    // CUDA + OPENCV2
    //////////////////
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/gpu.hpp>
#include <opencv2/gpu/gpu.hpp>

#endif // HAVE_OPENCV3
#else  // HAVE_CUDA

#ifdef HAVE_OPENCV3

    //////////////////
    // OPENCV3
    //////////////////
#include <opencv2/xfeatures2d.hpp>

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

namespace camodocal
{

/// @todo Replace with direct use of OpenCV3 APIs since the new APIs are finally consistent so multiple algorithms can be used at runtime easily.
class ORBGPU
{

#ifdef HAVE_CUDA
#ifdef HAVE_OPENCV3

    //////////////////
    // CUDA + OPENCV3
    //////////////////
    typedef cv::cuda::ORB                                  ORBType;
    typedef cv::cuda::GpuMat                               MatType;
    typedef cv::cuda::DescriptorMatcher                    MatcherType;
#else // HAVE_OPENCV3

    //////////////////
    // CUDA + OPENCV2
    //////////////////
    typedef cv::gpu::ORB_GPU                               ORBType;
    typedef cv::gpu::GpuMat                                MatType;
    typedef cv::gpu::BruteForceMatcher_GPU<cv::Hamming >   MatcherType;
#endif // HAVE_OPENCV3
#else // HAVE_CUDA
    
#ifdef HAVE_OPENCV3
    //////////////////
    // OPENCV3
    //////////////////
    typedef cv::ORB                                        ORBType;
    typedef cv::Mat                                        MatType;
    typedef cv::DescriptorMatcher                          MatcherType;
    
#else // HAVE_OPENCV3

    //////////////////
    // OPENCV2
    //////////////////
    typedef cv::ORB                                        ORBType;
    typedef cv::Mat                                        MatType;
    typedef cv::BruteForceMatcher<cv::Hamming >            MatcherType;
#endif // HAVE_OPENCV3
#endif // HAVE_CUDA

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

    cv::Ptr<ORBType> mORB_GPU;
    MatType mImageGPU;
    MatType mMaskGPU;
    MatType mKptsGPU;
    MatType mDtorsGPU;

    cv::Ptr<MatcherType> mMatcher;

    MatType mMatchMaskGPU;
    MatType mQDtorsGPU, mTDtorsGPU;

    boost::mutex mORBMutex;
    boost::mutex mMatchMutex;
    
    
//    cv::gpu::ORB_GPU mORB_GPU;
//    cv::gpu::GpuMat mImageGPU;
//    cv::gpu::GpuMat mMaskGPU;
//    cv::gpu::GpuMat mKptsGPU;
//    cv::gpu::GpuMat mDtorsGPU;
//
//    cv::gpu::BruteForceMatcher_GPU<cv::Hamming> mMatcher;
//
//    cv::gpu::GpuMat mMatchMaskGPU;
//    cv::gpu::GpuMat mQDtorsGPU, mTDtorsGPU;
//
//    boost::mutex mORBMutex;
//    boost::mutex mMatchMutex;
};

}

#endif
