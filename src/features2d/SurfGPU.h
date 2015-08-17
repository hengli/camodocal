#ifndef SURFGPU_H
#define SURFGPU_H

#include <boost/thread.hpp>

#ifdef HAVE_CUDA
#ifdef HAVE_OPENCV3

    //////////////////
    // CUDA + OPENCV3
    //////////////////
#include <opencv2/cudafeatures2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/cuda.hpp>
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


//#ifdef HAVE_CUDA
//    cv::gpu::SURF_GPU m_surfGPU;
//    cv::gpu::BruteForceMatcher_GPU<cv::L2<float> > m_matcher;
//#else
//    cv::SURF m_surfGPU;
//    cv::BruteForceMatcher<cv::L2<float> > m_matcher;
//#endif // HAVE_CUDA

/// @todo Replace with direct use of OpenCV3 APIs since the new APIs are finally consistent so multiple algorithms can be used at runtime easily.
class SurfGPU
{

#ifdef HAVE_CUDA
#ifdef HAVE_OPENCV3

    //////////////////
    // CUDA + OPENCV3
    //////////////////
    typedef cv::cuda::SURF_CUDA                            SURFType;
    typedef cv::cuda::GpuMat                               MatType;
    typedef cv::cuda::DescriptorMatcher                    MatcherType;
#else // HAVE_OPENCV3

    //////////////////
    // CUDA + OPENCV2
    //////////////////
    typedef cv::gpu::SURF_GPU                              SURFType;
    typedef cv::gpu::GpuMat                                MatType;
    typedef cv::gpu::BruteForceMatcher_GPU<cv::L2<float> > MatcherType;
#endif // HAVE_OPENCV3
#else // HAVE_CUDA
    
#ifdef HAVE_OPENCV3
    //////////////////
    // OPENCV3
    //////////////////
    typedef cv::xfeatures2d::SURF                          SURFType;
    typedef cv::Mat                                        MatType;
    typedef cv::DescriptorMatcher                          MatcherType;
    
#else // HAVE_OPENCV3

    //////////////////
    // OPENCV2
    //////////////////
    typedef cv::SURF                                       SURFType;
    typedef cv::Mat                                        MatType;
    typedef cv::BruteForceMatcher<cv::L2<float> >          MatcherType;
#endif // HAVE_OPENCV3
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

    cv::Ptr<SURFType>    m_surfGPU;
    cv::Ptr<MatcherType> m_matcher;
};

}

#endif
